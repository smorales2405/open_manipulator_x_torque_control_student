// ============================================================================
//  gz_io_control_node_student.cpp
//  Plantilla: Controlador Input-Output Linearization — OpenMANIPULATOR-X (simulacion Gazebo)
//
//  Salida de tarea:
//    y = [x, y, z, phi]^T
//    donde (x,y,z) es la posicion cartesiana del efector final y
//    phi = q2 + q3 + q4  es el angulo de inclinacion analitico del efector final.
//
//  Ley de control (a implementar):
//    v     = yddot_des + Kp_y*(y_des - y) + Kd_y*(ydot_des - ydot)
//    qddot = J4_dls^+ * (v - Jdot*qdot)     (pseudo-inversa DLS)
//    tau   = M(q) * qddot + b(q,dq)
//
//  Se debe completar las secciones marcadas con COMPLETAR:
//    SECCION 1 — Ganancias y parametros del controlador
//    SECCION 2 — Trayectoria de referencia cartesiana
//    SECCION 3 — Ley de control
//
//  Infraestructura proporcionada (NO modificar):
//    - Carga del modelo Pinocchio / URDF
//    - Cinematica directa: posicion (x,y,z), angulo phi, Jacobiano J4
//    - Termino de bias Jdot*qdot (aceleracion clasica del efector con qddot = 0)
//    - Calculo de M(q) y NLE(q,dq) mediante Pinocchio
//    - Feedforward de la friccion del URDF (siempre activo, tras la ley de
//      control): Pinocchio NO incluye <dynamics damping/friction> en M/NLE,
//      pero Gazebo SI simula esa friccion; el termino de Coulomb usa la
//      velocidad articular deseada dq_des = J4⁺·ydot_des
//    - Transicion inicial suave (polinomio de 5to orden): y0 → Y_START
//    - Captura automatica de la pose inicial del robot
//    - Suscriptor /joint_states  →  q, dq
//    - Publicador /arm_effort_controller/commands  →  tau
//    - Retorno suave (5to orden) a la pose inicial al cumplirse t_run + pausa
//      en reposo antes de soltar el torque: el robot queda en la pose de
//      partida y el nodo se puede volver a ejecutar sobre el mismo Gazebo
//    - Escritura de CSV:  gz_io_data_<test_num>.csv (sin el retorno final)
//    - Timer de control a 200 Hz (igual que los nodos hw_)
//    - Parametros ROS:  test_num, t_run
//
//  Ejemplo de ejecucion (con la simulacion Gazebo ya lanzada):
//    ros2 launch open_manipulator_x_torque_control torque_sim.launch.py
//    ros2 run open_manipulator_x_torque_control gz_io_control_node_student --ros-args -p test_num:=1 -p t_run:=15.0
// ============================================================================

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include <Eigen/Dense>   // Core + descomposiciones (antes llegaban via Pinocchio)
#include <Eigen/LU>
#include "omx/robot_model.hpp"
#include "omx/ros_utils.hpp"
#include "omx/trajectory.hpp"

using namespace std::chrono_literals;

#ifndef PACKAGE_DATA_DIR
#define PACKAGE_DATA_DIR "."
#endif
#ifndef PACKAGE_URDF_DIR
#define PACKAGE_URDF_DIR "."
#endif

static constexpr double PI   = M_PI;
static constexpr int    NARM = 4;

static constexpr char EFF_FRAME[] = "end_effector_link";

// Suavizado del tanh en el feedforward de friccion de Coulomb [rad/s]
// (usa la velocidad DESEADA, senal sin ruido: eps pequeno es seguro)
static constexpr double FRIC_EPS = 0.05;

// Duracion del retorno final a la pose inicial y de la pausa en reposo antes
// de soltar el torque [s] (infraestructura)
static constexpr double RETURN_TIME_S = 3.0;
static constexpr double HOLD_TIME_S   = 0.5;

// ═══════════════════════════════════════════════════════════════════════════
//  SECCION 1 — TRAYECTORIA DE REFERENCIA CARTESIANA
//
//  COMPLETAR: Definir y_des(t'), ydot_des(t') e yddot_des(t').
//
//  Parametro de entrada:
//    tp  — tiempo relativo al inicio de la fase circular [s]  (tp = t - T_TRANS)
//
//  Salida esperada (llenar ref.y, ref.ydot, ref.yddot):
//    ref.y    = [x_des(tp), y_des(tp), z_des(tp), phi_des(tp)]
//    ref.ydot = primera derivada temporal de ref.y (analitica)
//    ref.yddot= segunda derivada temporal de ref.y (analitica)
//
//  Parametros configurables (ajustar si es necesario):
//    T_TRANS — duracion de la fase de transicion inicial [s]
// ═══════════════════════════════════════════════════════════════════════════

static constexpr double T_TRANS   = 3.0;    // duracion de la transicion inicial  [s]
static const Eigen::Vector4d Y_START {0.0, 0.0, 0.0, 0.0};  // COMPLETAR 1b  [x, y, z, phi]

// ── Estructura de referencia cartesiana ─────────────────────────────────────
using omx::CartRef;     // {y, ydot, yddot}, y = [x, y, z, phi]
// ═══════════════════════════════════════════════════════════════════════════
static CartRef circularTrajectory(double t)
{

  CartRef ref;

  // -------------------------------------------------------------------
  // COMPLETAR: implementar aqui
  // -------------------------------------------------------------------

  (void)t;
  const double w = 1.0;
  (void)w;

  ref.y.setZero();
  ref.ydot.setZero();
  ref.yddot.setZero();

  return ref;
}
// ═══════════════════════════════════════════════════════════════════════════

// ═══════════════════════════════════════════════════════════════════════════

// ═══════════════════════════════════════════════════════════════════════════
//  SECCION 2 — GANANCIAS Y PARAMETROS DEL CONTROLADOR
//
//  COMPLETAR 1a: Asignar las ganancias cartesianas Kp_y y Kd_y.
//    Indice:  [x, y, z, phi]
//    Referencia de disenio:
//      - Sistema de 2do orden desacoplado: wn = sqrt(Kp_y),  zeta = Kd_y/(2*wn)
//      - Para amortiguamiento critico:     Kd_y = 2*sqrt(Kp_y)
//      - La frecuencia natural debe ser >> frecuencia de la trayectoria
//
//  COMPLETAR 1b: Definir Y_START — punto inicial de la trayectoria circular.
//    Debe coincidir con ref.y en t'=0 (i.e., circularTrajectory(0)).
//    El robot parte de su pose Gazebo y transiciona hacia Y_START en T_TRANS s.
//
//  Parametros configurables (ajustar si es necesario):
//    TAU_MAX — saturacion de torque por articulacion [N·m]  
//    LAMBDA  — factor de amortiguamiento DLS (rango tipico: 0.01 – 0.05)
// ═══════════════════════════════════════════════════════════════════════════

static const Eigen::Vector4d KP_Y = {0.0, 0.0, 0.0, 0.0};   // COMPLETAR [x, y, z, phi]
static const Eigen::Vector4d KD_Y = {0.0, 0.0, 0.0, 0.0};   // COMPLETAR [x, y, z, phi]

static constexpr double TAU_MAX   = 0.0;   // COMPLETAR limite de torque por articulacion [N·m]
static constexpr double LAMBDA    = 0.01;  // factor de amortiguamiento DLS (rango tipico: 0.01-0.05)
// ═══════════════════════════════════════════════════════════════════════════

// ── Nodo de control ──────────────────────────────────────────────────────────
class IOControlNode : public rclcpp::Node
{
public:
  IOControlNode()
  : Node("gz_io_control_node"), t_(0.0), y0_initialized_(false)
  {
    this->declare_parameter<int>("test_num", 1);
    const int test_num = this->get_parameter("test_num").as_int();

    this->declare_parameter<double>("t_run", 0.0);
    t_run_ = this->get_parameter("t_run").as_double();

    // ── Carga del modelo Pinocchio ─────────────────────────────────────────
    const std::string urdf = std::string(PACKAGE_URDF_DIR) + "/open_manipulator_x.urdf";
    try {
      model_.load(urdf, EFF_FRAME);
    } catch (const std::exception & e) {
      RCLCPP_FATAL(this->get_logger(), "Pinocchio: %s", e.what());
      throw;
    }
    // Friccion articular del URDF (<dynamics damping/friction>), parseada por
    // Pinocchio pero NO incluida en M/nle: usada por el feedforward (infra).
    fric_damping_ = model_.urdfDamping();
    fric_coulomb_ = model_.urdfFriction();

    RCLCPP_INFO(this->get_logger(),
      "Modelo cargado: nv=%d  frame='%s'",
      model_.nv(), EFF_FRAME);
    RCLCPP_INFO(this->get_logger(),
      "Feedforward friccion URDF: damping=[%.4f %.4f %.4f %.4f]  coulomb=[%.4f %.4f %.4f %.4f]",
      fric_damping_[0], fric_damping_[1], fric_damping_[2], fric_damping_[3],
      fric_coulomb_[0], fric_coulomb_[1], fric_coulomb_[2], fric_coulomb_[3]);
    RCLCPP_INFO(this->get_logger(),
      "Kp_y=[%.1f %.1f %.1f %.1f]  Kd_y=[%.1f %.1f %.1f %.1f]  tau_max=%.2f N·m",
      KP_Y[0], KP_Y[1], KP_Y[2], KP_Y[3],
      KD_Y[0], KD_Y[1], KD_Y[2], KD_Y[3], TAU_MAX);
    RCLCPP_INFO(this->get_logger(),
      "lambda=%.3f  T_trans=%.1f s", LAMBDA, T_TRANS);
    if (t_run_ > 0.0) {
      RCLCPP_INFO(this->get_logger(), "Tiempo de simulacion: %.1f s", t_run_);
    }

    open_csv(test_num);

    torque_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/arm_effort_controller/commands", 10);

    // Lazo de control a 200 Hz (dt = 5 ms), igual que el nodo hw
    timer_ = this->create_wall_timer(5ms, [this]() { tick(); });
  }

  ~IOControlNode()
  {
    if (csv_.is_open()) {
      csv_.close();
      RCLCPP_INFO(this->get_logger(), "CSV cerrado: %s", csv_path_.c_str());
    }
  }

private:
  // ── CSV ───────────────────────────────────────────────────────────────────
  void open_csv(int test_num)
  {
    csv_path_ = std::string(PACKAGE_DATA_DIR) + "/feedback_linearization/sim/act2/gz_io_data_"
                + std::to_string(test_num) + ".csv";
    omx::openCsv(csv_, csv_path_,
      "t,"
      "q1,q2,q3,q4,"
      "x,y,z,phi,"
      "x_des,y_des,z_des,phi_des,"
      "xdot,ydot,zdot,phidot,"
      "xdot_des,ydot_des,zdot_des,phidot_des,"
      "tau1,tau2,tau3,tau4",
      this->get_logger());
  }

  // ── Tick de control a 200 Hz ──────────────────────────────────────────────
  void tick()
  {
    if (!js_.ready()) { return; }

    // ── 1. Leer estado articular ───────────────────────────────────────────
    Eigen::Vector4d q, dq;
    js_.read(q, dq);

    // ── 2. Cinematica directa y Jacobiano ─────────────────────────────────
    // eeKinematics: posicion p, jacobiano lineal J (3×4, filas {vx,vy,vz}) y
    // bias J̇·q̇ del efector, en marco LOCAL_WORLD_ALIGNED.
    const auto ee = model_.eeKinematics(q, dq);   // p, J lineal y J̇·q̇ del efector

    // Posicion del efector final
    const Eigen::Vector3d p = ee.p;
    const double phi = q[1] + q[2] + q[3];  // analitico: phi = q2 + q3 + q4

    // Jacobiano de tarea 4×4: filas {vx, vy, vz, dphi/dq} — columnas {j1..j4}
    Eigen::Matrix4d J4;
    J4.topRows<3>() = ee.J;
    J4.row(3) << 0.0, 1.0, 1.0, 1.0;   // dphi/dq = [0,1,1,1] (constante analitico)

    // Velocidad de la salida de tarea actual:  ydot = J4 * dq
    const Eigen::Vector4d ydot = J4 * dq;

    // ── 3. Termino de bias  Jdot*qdot ─────────────────────────────────────
    // Se obtiene como la aceleracion clasica del frame con qddot = 0:
    //   a_clasica(q, dq, 0) = Jdot(q,dq) * dq
    Eigen::Vector4d jdqd;
    jdqd << ee.dJdq, 0.0;   // Jdot_phi * qdot = 0 (fila [0,1,1,1] es constante)

    // ── 4. Dinamica: M(q) y b(q,dq) ──────────────────────────────────────
    const Eigen::Matrix4d M4   = model_.massMatrix(q);
    const Eigen::Vector4d nle4 = model_.nonlinearEffects(q, dq);

    // ── 5. Salida de tarea y captura de condicion inicial ─────────────────
    const Eigen::Vector4d y { p[0], p[1], p[2], phi };

    if (!y0_initialized_) {
      y0_ = y;
      y0_initialized_ = true;
      RCLCPP_INFO(this->get_logger(),
        "Pose inicial capturada: x=%.3f  y=%.3f  z=%.3f  phi=%.3f rad",
        y0_[0], y0_[1], y0_[2], y0_[3]);
    }

    // ── 6. Referencia segun fase ──────────────────────────────────────────
    // [0, T_TRANS)                  : transicion suave desde la pose inicial hasta Y_START
    // [T_TRANS, t_run)              : trayectoria definida en circularTrajectory()
    // [t_run, t_run+RETURN_TIME_S)  : retorno suave a la pose inicial y0
    // [t_run+RETURN_TIME_S, ...)    : pausa en reposo en y0 (luego torque cero)
    CartRef ref;
    if (t_ < T_TRANS) {
      ref = omx::quinticTransition(t_, T_TRANS, y0_, CartRef{Y_START, Eigen::Vector4d::Zero(), Eigen::Vector4d::Zero()});
    } else if (t_run_ <= 0.0 || t_ < t_run_) {
      ref = circularTrajectory(t_ - T_TRANS);
    } else if (t_ < t_run_ + RETURN_TIME_S) {
      if (!return_logged_) {
        RCLCPP_INFO(this->get_logger(),
          "Iniciando retorno cartesiano a y_inicial (t=%.1fs, dur=%.1fs)", t_, RETURN_TIME_S);
        return_logged_ = true;
      }
      ref = omx::quinticReturn(t_ - t_run_, RETURN_TIME_S, circularTrajectory(t_run_ - T_TRANS), y0_);
    } else {
      ref.y = y0_;
      ref.ydot.setZero();
      ref.yddot.setZero();
    }

    // ══════════════════════════════════════════════════════════════════════
    //  SECCION 3 — LEY DE CONTROL  (COMPLETAR)
    //
    //  Disponible:
    //    y      (4×1)  — salida de tarea actual        [x, y, z, phi]
    //    ydot   (4×1)  — velocidad de la salida actual
    //    ref.y, ref.ydot, ref.yddot  — referencia y sus derivadas
    //    J4     (4×4)  — Jacobiano de tarea
    //    jdqd   (4×1)  — termino Jdot(q,dq)*dq  (bias de aceleracion)
    //    M4     (4×4)  — matriz de masa inercial  M(q)
    //    nle4   (4×1)  — efectos no lineales      b(q,dq)
    //    KP_Y, KD_Y        — ganancias (definidas en Seccion 1)
    //    LAMBDA        — lambda para la pseudo-inversa DLS
    //    TAU_MAX       — saturacion de torque
    //
    //  Ley de control IO Linearization:
    //    ey     = ref.y - y
    //    eydot  = ref.ydot - ydot
    //    v      = ref.yddot + Kp_y*ey + Kd_y*eydot
    //    A      = J4*J4^T + lambda^2 * I          (matriz auxiliar DLS)
    //    qddot  = J4^T * A^-1 * (v - Jdot*dq)    (pseudo-inversa DLS)
    //    tau    = M(q) * qddot + b(q,dq)
    //    tau_sat = clamp(tau, -TAU_MAX, TAU_MAX)
    // ══════════════════════════════════════════════════════════════════════

    (void)M4; (void)nle4;   // suprimir warnings mientras la seccion esta incompleta
    Eigen::Vector4d tau_sat = Eigen::Vector4d::Zero();   // <-- reemplazar con implementacion

    // ══════════════════════════════════════════════════════════════════════

    // ── Feedforward de friccion del URDF (infraestructura, NO modificar) ───
    //  Gazebo simula la friccion articular del URDF, pero Pinocchio no la
    //  incluye en M/nle: se compensa aqui, fuera de la ley de control.
    //  Viscosa con la velocidad medida; Coulomb con la velocidad articular
    //  DESEADA dq_des = J4⁺·ydot_des (senal sin ruido). Tras sumarla se
    //  re-satura a TAU_MAX.
    {
      const Eigen::Matrix4d A_fric =
        J4 * J4.transpose() + LAMBDA * LAMBDA * Eigen::Matrix4d::Identity();
      const Eigen::Vector4d dq_des = J4.transpose() * A_fric.ldlt().solve(ref.ydot);
      for (int i = 0; i < NARM; ++i) {
        tau_sat[i] += fric_damping_[i] * dq[i]
                    + fric_coulomb_[i] * std::tanh(dq_des[i] / FRIC_EPS);
      }
      tau_sat = tau_sat.cwiseMin(TAU_MAX).cwiseMax(-TAU_MAX);
    }

    // ── 7. Publicar torques ────────────────────────────────────────────────
    std_msgs::msg::Float64MultiArray cmd;
    cmd.data.assign(tau_sat.data(), tau_sat.data() + NARM);
    torque_pub_->publish(cmd);

    const char * phase = (t_ < T_TRANS) ? "TRANS" :
                         (t_run_ <= 0.0 || t_ < t_run_) ? "CIRC " :
                         (t_ < t_run_ + RETURN_TIME_S)  ? "RET  " : "HOME ";
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "[%s] t=%.2fs  y=[%.3f %.3f %.3f %.3f]  tau=[%.3f %.3f %.3f %.3f]",
      phase, t_, y[0], y[1], y[2], y[3],
      tau_sat[0], tau_sat[1], tau_sat[2], tau_sat[3]);

    // ── 8. CSV (excluye el retorno final y la pausa en reposo) ────────────
    const bool in_return_or_hold = (t_run_ > 0.0 && t_ >= t_run_);
    if (csv_.is_open() && !in_return_or_hold) {
      csv_ << std::fixed << std::setprecision(6)
           << t_
           << ',' << q[0]           << ',' << q[1]           << ',' << q[2]           << ',' << q[3]
           << ',' << y[0]           << ',' << y[1]           << ',' << y[2]           << ',' << y[3]
           << ',' << ref.y[0]       << ',' << ref.y[1]       << ',' << ref.y[2]       << ',' << ref.y[3]
           << ',' << ydot[0]        << ',' << ydot[1]        << ',' << ydot[2]        << ',' << ydot[3]
           << ',' << ref.ydot[0]    << ',' << ref.ydot[1]    << ',' << ref.ydot[2]    << ',' << ref.ydot[3]
           << ',' << tau_sat[0]     << ',' << tau_sat[1]     << ',' << tau_sat[2]     << ',' << tau_sat[3]
           << '\n';
    }

    t_ += 0.005;   // Ts del lazo de 200 Hz

    // ── 9. Fin: tras el retorno y la pausa se suelta el torque (el robot ──
    //       queda en la pose de partida, listo para otra ejecucion)
    if (t_run_ > 0.0 && t_ >= t_run_ + RETURN_TIME_S + HOLD_TIME_S) {
      RCLCPP_INFO(this->get_logger(),
        "Retorno a y_inicial completado (t=%.1f s). Deteniendo control.", t_);
      std_msgs::msg::Float64MultiArray zero;
      zero.data.assign(NARM, 0.0);
      torque_pub_->publish(zero);
      if (csv_.is_open()) {
        csv_.close();
        RCLCPP_INFO(this->get_logger(), "CSV cerrado: %s", csv_path_.c_str());
      }
      timer_->cancel();
    }
  }

  // ── Miembros ──────────────────────────────────────────────────────────────
  omx::RobotModel model_;

  double t_;
  double t_run_;

  Eigen::Vector4d fric_damping_{Eigen::Vector4d::Zero()};
  Eigen::Vector4d fric_coulomb_{Eigen::Vector4d::Zero()};

  Eigen::Vector4d y0_;           // pose cartesiana inicial capturada
  bool            y0_initialized_;
  bool            return_logged_{false};

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_pub_;
  omx::JointStateListener js_{*this};   // /joint_states (joint1..joint4 por nombre)
  rclcpp::TimerBase::SharedPtr timer_;

  std::ofstream csv_;
  std::string   csv_path_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IOControlNode>());
  rclcpp::shutdown();
  return 0;
}
