// ============================================================================
//  gz_fl_control_node_base.cpp
//  Plantilla: Controlador Feedback Linearization — OpenMANIPULATOR-X (simulacion Gazebo)
//
//  Ley de control (espacio articular):
//
//    tau = M(q) * [ ddq_des + Kp*(q_des - q) + Kd*(dq_des - dq) ] + NLE(q,dq)
//
//  Se debe completar las secciones marcadas con COMPLETAR:
//    SECCION 1 — Trayectoria de referencia articular
//    SECCION 2 — Ganancias del controlador
//    SECCION 3 — Ley de control
//
//  Infraestructura proporcionada (NO modificar):
//    - Carga del modelo Pinocchio / URDF
//    - Calculo de M(q) y NLE(q,dq) mediante Pinocchio
//    - Feedforward de la friccion del URDF (siempre activo, tras la ley de
//      control): Pinocchio NO incluye <dynamics damping/friction> en M/NLE,
//      pero Gazebo SI simula esa friccion; se compensa como infraestructura
//    - Suscriptor /joint_states  →  q, dq
//    - Publicador /arm_effort_controller/commands  →  tau
//    - Rampa quintica de 3 s desde la pose inicial medida hasta el inicio de
//      la trayectoria (continuidad de posicion, velocidad y aceleracion)
//    - Retorno quintico al reposo en q_inicial al cumplirse t_run + pausa en
//      reposo antes de soltar el torque: el robot queda en la pose de partida
//      y el nodo se puede volver a ejecutar sobre el mismo Gazebo
//    - Escritura de CSV:  gz_fl_data_<test_num>.csv (sin el retorno final)
//    - Timer de control a 100 Hz
//    - Parametros ROS:  test_num, t_run
//
//  Ejemplo de ejecucion (con la simulacion Gazebo ya lanzada):
//    ros2 launch open_manipulator_x_torque_control torque_sim.launch.py
//    ros2 run open_manipulator_x_torque_control gz_fl_control_node_base --ros-args -p test_num:=1 -p t_run:=15.0
// ============================================================================

#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include <Eigen/Core>
#include <pinocchio/multibody.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>

using namespace std::chrono_literals;

#ifndef PACKAGE_DATA_DIR
#define PACKAGE_DATA_DIR "."
#endif
#ifndef PACKAGE_URDF_DIR
#define PACKAGE_URDF_DIR "."
#endif

static constexpr double PI   = M_PI;
static constexpr int    NARM = 4;    // articulaciones controladas (joint1..joint4)

// Suavizado del tanh en el feedforward de friccion de Coulomb [rad/s]
// (usa la velocidad DESEADA, senal sin ruido: eps pequeno es seguro)
static constexpr double FRIC_EPS = 0.05;

// Duraciones de la rampa inicial, del retorno final a q_inicial y de la pausa
// en reposo antes de soltar el torque [s] (infraestructura)
static constexpr double RAMP_TIME_S   = 3.0;
static constexpr double RETURN_TIME_S = 3.0;
static constexpr double HOLD_TIME_S   = 0.5;

// ═══════════════════════════════════════════════════════════════════════════
//  SECCION 1 — TRAYECTORIA DE REFERENCIA ARTICULAR
//
// ── Estructura de referencia articular ──────────────────────────────────────
struct Reference {
  Eigen::Vector4d q;    // posicion deseada     [rad]
  Eigen::Vector4d dq;   // velocidad deseada    [rad/s]
  Eigen::Vector4d ddq;  // aceleracion deseada  [rad/s²]
};
//  COMPLETAR: Definir q_des(t), dq_des(t) y ddq_des(t) para cada articulacion.
//
//  Parametro de entrada:
//    t  — tiempo actual de simulacion [s]
//
//  Salida esperada (llenar ref.q, ref.dq, ref.ddq):
//    ref.q   << q1_des(t),   q2_des(t),   q3_des(t),   q4_des(t);
//    ref.dq  << dq1_des(t),  dq2_des(t),  dq3_des(t),  dq4_des(t);
//    ref.ddq << ddq1_des(t), ddq2_des(t), ddq3_des(t), ddq4_des(t);
//
//  Nota: esta funcion se usa para t >= RAMP_TIME_S. Para t < RAMP_TIME_S
//        se aplica automaticamente la rampa quintica desde la pose inicial
//        hasta desiredTrajectory(RAMP_TIME_S) (ver quinticTransition).
//
// ═══════════════════════════════════════════════════════════════════════════

static Reference desiredTrajectory(double t)
{
  Reference ref;

  // -------------------------------------------------------------------
  // COMPLETAR: implementar aqui
  // -------------------------------------------------------------------

  (void)t;
  const double w = 1.0;
  (void)w;

  ref.q.setZero();
  ref.dq.setZero();
  ref.ddq.setZero();

  return ref;
}

// ═══════════════════════════════════════════════════════════════════════════

// ── Transiciones quinticas (INFRAESTRUCTURA — no es necesario modificar) ────
// Polinomio quintico general: interpola (q,dq,ddq) desde el estado de borde
// (q0,v0,a0) en t=0 hasta (qf,vf,af) en t=T.
static Reference quinticBlend(
  double t, double T,
  const Eigen::Vector4d & q0, const Eigen::Vector4d & v0, const Eigen::Vector4d & a0,
  const Eigen::Vector4d & qf, const Eigen::Vector4d & vf, const Eigen::Vector4d & af)
{
  if (T <= 0.0) {
    Reference ref;
    ref.q = qf; ref.dq = vf; ref.ddq = af;
    return ref;
  }

  const double T2 = T*T, T3 = T2*T, T4 = T3*T, T5 = T4*T;
  const Eigen::Vector4d c0 = q0;
  const Eigen::Vector4d c1 = v0;
  const Eigen::Vector4d c2 = 0.5 * a0;
  const Eigen::Vector4d c3 = (20.0*(qf-q0) - (8.0*vf+12.0*v0)*T - (3.0*a0-af)*T2) / (2.0*T3);
  const Eigen::Vector4d c4 = (30.0*(q0-qf) + (14.0*vf+16.0*v0)*T + (3.0*a0-2.0*af)*T2) / (2.0*T4);
  const Eigen::Vector4d c5 = (12.0*(qf-q0) - (6.0*vf+6.0*v0)*T - (a0-af)*T2) / (2.0*T5);

  const double t2 = t*t, t3 = t2*t, t4 = t3*t, t5 = t4*t;
  Reference ref;
  ref.q   = c0 + c1*t  + c2*t2  + c3*t3  + c4*t4  + c5*t5;
  ref.dq  = c1 + 2.0*c2*t + 3.0*c3*t2 + 4.0*c4*t3 + 5.0*c5*t4;
  ref.ddq = 2.0*c2 + 6.0*c3*t + 12.0*c4*t2 + 20.0*c5*t3;
  return ref;
}

// Rampa inicial: parte en reposo en q0 y llega a la trayectoria en t = T, con
// velocidad y aceleracion continuas en el empalme.
static Reference quinticTransition(double t, double T, const Eigen::Vector4d & q0)
{
  const Reference target = desiredTrajectory(T);
  return quinticBlend(t, T, q0, Eigen::Vector4d::Zero(), Eigen::Vector4d::Zero(),
                      target.q, target.dq, target.ddq);
}

// Retorno final: parte del estado de la trayectoria al cumplirse t_run
// (start = desiredTrajectory(t_run)) y llega en reposo a qf (q_inicial).
static Reference quinticReturn(double t, double T, const Reference & start,
                               const Eigen::Vector4d & qf)
{
  return quinticBlend(t, T, start.q, start.dq, start.ddq,
                      qf, Eigen::Vector4d::Zero(), Eigen::Vector4d::Zero());
}

// ═══════════════════════════════════════════════════════════════════════════
//  SECCION 2 — GANANCIAS DEL CONTROLADOR
//
//  COMPLETAR: Asignar los valores de Kp, Kd y el limite de torque.
//
//  Indice de cada elemento:  [joint1, joint2, joint3, joint4]
//  Referencia de diseño:
//    - Sistema de 2do orden desacoplado: wn = sqrt(Kp),  zeta = Kd / (2*wn)
//    - Para amortiguamiento critico:     Kd = 2*sqrt(Kp)
// ═══════════════════════════════════════════════════════════════════════════

static const Eigen::Vector4d KP = {0.0, 0.0, 0.0, 0.0};   // COMPLETAR
static const Eigen::Vector4d KD = {0.0, 0.0, 0.0, 0.0};   // COMPLETAR
static constexpr double TAU_MAX = 0.0;                      // COMPLETAR  [N·m]

// ═══════════════════════════════════════════════════════════════════════════

// ── Nodo de control ──────────────────────────────────────────────────────────
class FLControlNode : public rclcpp::Node
{
public:
  FLControlNode()
  : Node("gz_fl_control_node"), t_(0.0)
  {
    this->declare_parameter<int>("test_num", 1);
    const int test_num = this->get_parameter("test_num").as_int();

    this->declare_parameter<double>("t_run", 0.0);
    t_run_ = this->get_parameter("t_run").as_double();

    // ── Carga del modelo Pinocchio ─────────────────────────────────────────
    const std::string urdf = std::string(PACKAGE_URDF_DIR) + "/open_manipulator_x.urdf";
    try {
      pinocchio::urdf::buildModel(urdf, model_);
    } catch (const std::exception & e) {
      RCLCPP_FATAL(this->get_logger(), "Pinocchio: %s", e.what());
      throw;
    }
    data_ = pinocchio::Data(model_);

    // Friccion articular del URDF (<dynamics damping/friction>), parseada por
    // Pinocchio pero NO incluida en M/nle: usada por el feedforward (infra).
    fric_damping_ = model_.damping.head<NARM>();
    fric_coulomb_ = model_.friction.head<NARM>();

    RCLCPP_INFO(this->get_logger(), "Modelo cargado: nv=%d", model_.nv);
    RCLCPP_INFO(this->get_logger(),
      "Feedforward friccion URDF: damping=[%.4f %.4f %.4f %.4f]  coulomb=[%.4f %.4f %.4f %.4f]",
      fric_damping_[0], fric_damping_[1], fric_damping_[2], fric_damping_[3],
      fric_coulomb_[0], fric_coulomb_[1], fric_coulomb_[2], fric_coulomb_[3]);
    RCLCPP_INFO(this->get_logger(),
      "Kp=[%.1f %.1f %.1f %.1f]  Kd=[%.1f %.1f %.1f %.1f]  tau_max=%.2f N·m",
      KP[0], KP[1], KP[2], KP[3],
      KD[0], KD[1], KD[2], KD[3], TAU_MAX);
    if (t_run_ > 0.0) {
      RCLCPP_INFO(this->get_logger(), "Tiempo de simulacion: %.1f s", t_run_);
    } else {
      RCLCPP_INFO(this->get_logger(), "Tiempo de simulacion: ilimitado");
    }

    open_csv(test_num);

    torque_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/arm_effort_controller/commands", 10);

    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        last_js_ = msg;
      });

    timer_ = this->create_wall_timer(10ms, [this]() { tick(); });
  }

  ~FLControlNode()
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
    std::filesystem::create_directories(std::string(PACKAGE_DATA_DIR) + "/lab4/sim/act1");
    csv_path_ = std::string(PACKAGE_DATA_DIR) + "/lab4/sim/act1/gz_fl_data_"
                + std::to_string(test_num) + ".csv";
    csv_.open(csv_path_);
    if (!csv_.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "No se pudo crear: %s", csv_path_.c_str());
      return;
    }
    csv_ << "t,"
         << "q1,q2,q3,q4,"
         << "dq1,dq2,dq3,dq4,"
         << "q1_des,q2_des,q3_des,q4_des,"
         << "dq1_des,dq2_des,dq3_des,dq4_des,"
         << "tau1,tau2,tau3,tau4\n";
    RCLCPP_INFO(this->get_logger(), "CSV: %s", csv_path_.c_str());
  }

  // ── Lectura de estados articulares ──
  void read_js(Eigen::Vector4d & q, Eigen::Vector4d & dq)
  {
    static const std::array<std::string, NARM> names = {
      "joint1", "joint2", "joint3", "joint4"
    };
    q.setZero();
    dq.setZero();
    if (!last_js_) { return; }
    const auto & js = *last_js_;
    for (int j = 0; j < NARM; ++j) {
      for (std::size_t i = 0; i < js.name.size(); ++i) {
        if (js.name[i] == names[j]) {
          if (i < js.position.size()) { q[j]  = js.position[i]; }
          if (i < js.velocity.size()) { dq[j] = js.velocity[i]; }
          break;
        }
      }
    }
  }

  // ── Tick de control a 100 Hz ──────────────────────────────────────────────
  void tick()
  {
    if (!last_js_) { return; }

    // ── 1. Leer estado articular ───────────────────────────────────────────
    Eigen::Vector4d q, dq;
    read_js(q, dq);

    // Captura de la pose inicial (primer tick con /joint_states valido)
    if (!q_initial_captured_) {
      q_initial_ = q;
      q_initial_captured_ = true;
      RCLCPP_INFO(this->get_logger(), "q_inicial=[%.3f %.3f %.3f %.3f] rad",
        q[0], q[1], q[2], q[3]);
    }

    // Referencia: rampa quintica inicial + trayectoria deseada + retorno
    // quintico a q_inicial + pausa en reposo (infraestructura)
    Reference ref;
    if (t_ < RAMP_TIME_S) {
      ref = quinticTransition(t_, RAMP_TIME_S, q_initial_);
    } else if (t_run_ <= 0.0 || t_ < t_run_) {
      ref = desiredTrajectory(t_);
    } else if (t_ < t_run_ + RETURN_TIME_S) {
      if (!return_logged_) {
        RCLCPP_INFO(this->get_logger(),
          "Iniciando retorno quintico a q_inicial (t=%.1fs, dur=%.1fs)", t_, RETURN_TIME_S);
        return_logged_ = true;
      }
      ref = quinticReturn(t_ - t_run_, RETURN_TIME_S, desiredTrajectory(t_run_), q_initial_);
    } else {
      ref.q = q_initial_;
      ref.dq.setZero();
      ref.ddq.setZero();
    }

    // ── 2. Estado completo para Pinocchio (articulaciones de pinza = 0) ────
    Eigen::VectorXd q_pin  = Eigen::VectorXd::Zero(model_.nv);
    Eigen::VectorXd dq_pin = Eigen::VectorXd::Zero(model_.nv);
    q_pin.head<NARM>()  = q;
    dq_pin.head<NARM>() = dq;

    // ── 3. Matriz de masa  M(q)  [NARM x NARM] ────────────────────────────
    // crba() llena solo el triangulo superior.
    pinocchio::crba(model_, data_, q_pin);
    data_.M.triangularView<Eigen::StrictlyLower>() =
      data_.M.triangularView<Eigen::StrictlyUpper>().transpose();

    // ── 4. Efectos no lineales  b(q,dq) = C(q,dq)*dq + g(q)  [NARM x 1] ──
    pinocchio::nonLinearEffects(model_, data_, q_pin, dq_pin);

    const Eigen::Matrix4d M   = data_.M.topLeftCorner<NARM, NARM>();
    const Eigen::Vector4d nle = data_.nle.head<NARM>();

    // ══════════════════════════════════════════════════════════════════════
    //  SECCION 3 — LEY DE CONTROL  (COMPLETAR)
    //
    //  Disponible:
    //    q, dq                    — posicion y velocidad articular medidas
    //    ref.q, ref.dq, ref.ddq   — referencia y sus derivadas analiticas
    //    M   (4×4)                — matriz de masa inercial  M(q)
    //    nle (4×1)                — efectos no lineales      b(q,dq)
    //    KP, KD                   — ganancias (definidas en Seccion 1)
    //    TAU_MAX                  — saturacion de torque
    //
    //  Ley de control Feedback Linearization:
    //    e       = ref.q - q
    //    de      = ref.dq - dq
    //    v       = ref.ddq + Kp*e + Kd*de
    //    tau     = M(q) * v + b(q,dq)
    //    tau_sat = clamp(tau, -TAU_MAX, TAU_MAX)
    // ══════════════════════════════════════════════════════════════════════

    (void)M; (void)nle;   // suprimir warnings mientras la seccion esta incompleta
    Eigen::Vector4d tau_sat = Eigen::Vector4d::Zero();   // <-- reemplazar con implementacion

    // ══════════════════════════════════════════════════════════════════════

    // ── Feedforward de friccion del URDF (infraestructura, NO modificar) ───
    //  Gazebo simula la friccion articular del URDF, pero Pinocchio no la
    //  incluye en M/nle: se compensa aqui, fuera de la ley de control.
    //  Viscosa con la velocidad medida; Coulomb con la velocidad DESEADA
    //  (senal sin ruido). Tras sumarla se re-satura a TAU_MAX.
    for (int i = 0; i < NARM; ++i) {
      tau_sat[i] += fric_damping_[i] * dq[i]
                  + fric_coulomb_[i] * std::tanh(ref.dq[i] / FRIC_EPS);
    }
    tau_sat = tau_sat.cwiseMin(TAU_MAX).cwiseMax(-TAU_MAX);

    // ── 5. Publicar torques ────────────────────────────────────────────────
    std_msgs::msg::Float64MultiArray cmd;
    cmd.data.assign(tau_sat.data(), tau_sat.data() + NARM);
    torque_pub_->publish(cmd);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "t=%.2fs  q=[%.3f %.3f %.3f %.3f]  tau=[%.3f %.3f %.3f %.3f] Nm",
      t_, q[0], q[1], q[2], q[3],
      tau_sat[0], tau_sat[1], tau_sat[2], tau_sat[3]);

    // ── 6. CSV (excluye el retorno final y la pausa en reposo) ────────────
    const bool in_return_or_hold = (t_run_ > 0.0 && t_ >= t_run_);
    if (csv_.is_open() && !in_return_or_hold) {
      csv_ << std::fixed << std::setprecision(6)
           << t_
           << ',' << q[0]       << ',' << q[1]       << ',' << q[2]       << ',' << q[3]
           << ',' << dq[0]      << ',' << dq[1]       << ',' << dq[2]      << ',' << dq[3]
           << ',' << ref.q[0]   << ',' << ref.q[1]   << ',' << ref.q[2]   << ',' << ref.q[3]
           << ',' << ref.dq[0]  << ',' << ref.dq[1]  << ',' << ref.dq[2]  << ',' << ref.dq[3]
           << ',' << tau_sat[0] << ',' << tau_sat[1] << ',' << tau_sat[2] << ',' << tau_sat[3]
           << '\n';
    }

    t_ += 0.01;

    // ── 7. Fin: tras el retorno y la pausa se suelta el torque (el robot ──
    //       queda en la pose de partida, listo para otra ejecucion)
    if (t_run_ > 0.0 && t_ >= t_run_ + RETURN_TIME_S + HOLD_TIME_S) {
      RCLCPP_INFO(this->get_logger(),
        "Retorno a q_inicial completado (t=%.1f s). Deteniendo control.", t_);
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
  pinocchio::Model  model_;
  pinocchio::Data   data_;
  double t_;
  double t_run_;

  Eigen::Vector4d fric_damping_{Eigen::Vector4d::Zero()};
  Eigen::Vector4d fric_coulomb_{Eigen::Vector4d::Zero()};

  Eigen::Vector4d q_initial_{Eigen::Vector4d::Zero()};
  bool q_initial_captured_{false};
  bool return_logged_{false};

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr  joint_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::JointState::SharedPtr last_js_;

  std::ofstream csv_;
  std::string   csv_path_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FLControlNode>());
  rclcpp::shutdown();
  return 0;
}
