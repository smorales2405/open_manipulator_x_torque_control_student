/*
 * hw_io_control_node_student.cpp
 * Plantilla: Controlador Input-Output Linearization cartesiana
 * OpenMANIPULATOR-X hardware real via Dynamixel SDK directo.
 *
 * Salida de tarea:  y = [x, y, z, phi]^T
 *   (x,y,z) posicion cartesiana del efector final (frame end_effector_link).
 *   phi = q2 + q3 + q4  angulo de inclinacion analitico.
 *
 * Ley de control a implementar:
 *   v     = yddot_des + Kp_y*(y_des - y) + Kd_y*(ydot_des - ydot)
 *   qddot = J4^T * (J4*J4^T + lambda^2*I)^-1 * (v - Jdot*dq)  [DLS]
 *   tau   = M(q)*qddot + nle(q, dq)
 *
 * Completar las tres secciones marcadas:
 *   SECCION 1 — Trayectoria deseada cartesiana (cartesianTrajectory)
 *   SECCION 2 — Ganancias del controlador      (KP_Y, KD_Y globales)
 *   SECCION 3 — Ley de control IO              (en compute_io_control)
 *
 * Infraestructura proporcionada (NO modificar):
 *   - Inicializacion Dynamixel SDK: apertura del puerto, modos, SyncRead/SyncWrite
 *   - Lectura de estado (q, dq, corriente medida) en UN solo SyncRead del bloque
 *     contiguo 126-135 (corriente+velocidad+posicion), a loop_rate_hz
 *   - Filtro alpha-beta de posicion/velocidad articular (ab_alpha, ab_beta)
 *   - Cinematica directa: posicion (x,y,z), angulo phi, Jacobiano J4 (4×4)
 *   - Termino de bias Jdot*dq (aceleracion clasica del efector con qddot = 0)
 *   - Calculo de M(q) y nle(q,dq) mediante Pinocchio (en compute_io_control)
 *   - Transicion inicial suave (5to orden): pose inicial medida → Y_START
 *   - Conversion torque -> corriente via modelo OLS identificado
 *     (motorXM430W350T_params.yaml); el termino de Coulomb usa la velocidad
 *     articular DESEADA qdot_des = J4+ * ydot_des (senal sin ruido: sin
 *     chattering y con empuje de despegue contra la friccion estatica)
 *   - Saturacion defensiva del torque a +/-tau_max antes de enviar corriente
 *   - Guardas de altura minima Z_MIN_FLOOR (referencia y efector medido)
 *   - Retorno cartesiano al reposo en y_inicial al completar t_run + pausa en
 *     reposo antes de cortar corriente (evita que el brazo caiga por
 *     gravedad al recibir torque cero de golpe en medio de la trayectoria)
 *   - Verificacion de limites articulares y de corriente medida (parada de emergencia)
 *   - Deteccion de overruns del lazo de control
 *   - Publicador /hw/joint_states y escritura de CSV (el retorno final y la
 *     pausa en reposo no se registran en el CSV)
 *
 * Parametros ROS 2 (--ros-args -p nombre:=valor):
 *   port_name              [string]   "/dev/ttyUSB0"
 *   gain_scale             [double]   1.0    (escala de ganancias)
 *   t_run                  [double]   20.0   (duracion; 0 = sin limite)
 *   log_id                 [int]      1
 *   ab_alpha               [double]   0.2    (filtro alpha-beta: ganancia de posicion)
 *   ab_beta                [double]   0.02   (filtro alpha-beta: ganancia de velocidad)
 *   friction_fc_scale      [double]   0.95   (fraccion de Fc compensada)
 *   loop_rate_hz           [double]   200.0  (frecuencia del lazo [Hz], acotada [50,400])
 *
 * Parametros del modelo identificado (config/motorXM430W350T_params.yaml):
 *   motor_alpha            [double[4]]   ticks/N·m
 *   motor_Fv               [double[4]]   ticks/(rad/s)
 *   motor_Fc               [double[4]]   ticks
 *   motor_I_offset         [double[4]]   ticks
 *   motor_epsilon_friction [double]      rad/s
 *
 * Ejemplo de ejecucion:
 *   ros2 run open_manipulator_x_torque_control hw_io_control_node_student \
 *     --ros-args -p gain_scale:=1.0 -p log_id:=1 -p t_run:=20.0
 */

#include <array>
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

#include <Eigen/Dense>
#include <Eigen/LU>

#include "omx/dynamixel_arm.hpp"
#include "omx/motor_model.hpp"
#include "omx/robot_model.hpp"
#include "omx/ros_utils.hpp"
#include "omx/trajectory.hpp"

#ifndef PACKAGE_DATA_DIR
#define PACKAGE_DATA_DIR "."
#endif
#ifndef PACKAGE_URDF_DIR
#define PACKAGE_URDF_DIR "."
#endif

using namespace std::chrono_literals;

// ============================================================
// Constantes (servos Dynamixel y unidades: omx/dynamixel_io.hpp)
// ============================================================

static constexpr double PI         = 3.14159265358979323846;
static constexpr int    NUM_JOINTS = 4;

// Calibracion, limites y modelo del motor: omx::declareHwConfig
// (config/motorXM430W350T_params.yaml como parametros ROS 2).

using Vec4 = Eigen::Matrix<double, NUM_JOINTS, 1>;

// ═══════════════════════════════════════════════════════════════════════════
//  SECCION 1 — TRAYECTORIA DESEADA CARTESIANA
//
//  Definir ref.y, ref.ydot y ref.yddot como funciones del tiempo t
//  (tiempo relativo al inicio de la fase de trayectoria: t' = t - T_TRANS).
//
//  Parametro de entrada:
//    t   — tiempo relativo desde el fin de la transicion [s]  (t' = t - T_TRANS)
//
//  Salida esperada:
//    ref.y    << x_des(t),    y_des(t),    z_des(t),    phi_des(t);
//    ref.ydot << xdot_des(t), ydot_des(t), zdot_des(t), phidot_des(t);
//    ref.yddot<< xddot_des(t),yddot_des(t),zddot_des(t),phiddot_des(t);
//
//  Nota: ref.y en t=0 debe coincidir con Y_START (punto de inicio de la
//        transicion quintica) para garantizar continuidad.
//        Verificar: cartesianTrajectory(0).y == Y_START
//
//  Parametros configurables:
//    T_TRANS — duracion de la fase de transicion inicial [s]
//
//  Restricciones de seguridad al diseniar la trayectoria:
//    - z_des(t) debe mantenerse SIEMPRE por encima de Z_MIN_FLOOR mas un
//      margen de tracking (~2.5 cm); la infraestructura detiene el nodo si
//      la referencia o el efector medido bajan de Z_MIN_FLOOR.
//    - Evitar extension maxima con z minimo simultaneos: verificar con IK
//      que la trayectoria completa respeta los limites articulares.
// ═══════════════════════════════════════════════════════════════════════════

static constexpr double T_TRANS       = 3.0;   // duracion de la transicion inicial [s]
static constexpr double RETURN_TIME_S = 3.0;   // duracion del retorno cartesiano a y_inicial
static constexpr double HOLD_TIME_S   = 0.5;   // pausa en reposo antes de cortar corriente

// Punto de inicio de la trayectoria cartesiana [x, y, z, phi] en t'=0
// Debe coincidir con cartesianTrajectory(0).
static const Eigen::Vector4d Y_START{0.0, 0.0, 0.0, 0.0};

// Altura minima del efector sobre la placa metalica de la base [m].
// Guarda de infraestructura: se verifica en cada tick para la referencia y
// para el efector medido (durante la transicion solo se exige no descender
// por debajo de la pose inicial, que puede arrancar plegada).
static constexpr double Z_MIN_FLOOR = 0.075;

static constexpr char EFF_FRAME_NAME[] = "end_effector_link";

// ── Estructura de referencia cartesiana ─────────────────────────────────────
using omx::CartRef;     // {y, ydot, yddot}, y = [x, y, z, phi]
// ═══════════════════════════════════════════════════════════════════════════
static CartRef cartesianTrajectory(double t)
{
  CartRef ref;

  // -- COMPLETAR --
  (void)t;
  const double w = 1.0;
  (void)w;

  ref.y.setZero();
  ref.ydot.setZero();
  ref.yddot.setZero();
  // ────────────────

  return ref;
}
// ═══════════════════════════════════════════════════════════════════════════

// ═══════════════════════════════════════════════════════════════════════════
//  SECCION 2 — GANANCIAS DEL CONTROLADOR
//
//  Definir KP_Y y KD_Y para cada componente de la salida: [x, y, z, phi]
//    KP_Y — ganancias proporcionales cartesianas  [adim]
//    KD_Y — ganancias derivativas cartesianas     [adim]
//
//  Referencia de disenio (sistema de 2do orden desacoplado):
//    wn   = sqrt(KP_Y)         — frecuencia natural
//    zeta = KD_Y / (2 * wn)   — coeficiente de amortiguamiento
//    Para amortiguamiento critico: KD_Y = 2 * sqrt(KP_Y)
//    La frecuencia natural debe ser >> frecuencia de la trayectoria
//
//  Sugerencias practicas para hardware:
//    - zeta ≈ 0.7 funciona bien: KD_Y = 1.4 * sqrt(KP_Y). Con KD_Y = 0 el
//      lazo de tarea es un oscilador puro (zeta = 0): vibrara en el robot.
//    - Las direcciones z y phi las dominan las munecas (inercia diminuta):
//      necesitan KP_Y bastante mayores que x/y para vencer la friccion
//      estatica (de lo contrario quedan "pegadas" con error grande).
// ═══════════════════════════════════════════════════════════════════════════
static const Eigen::Vector4d KP_Y = {0.0 /* kpx */, 0.0 /* kpy */, 0.0 /* kpz */, 0.0 /* kpphi */};
static const Eigen::Vector4d KD_Y = {0.0 /* kdx */, 0.0 /* kdy */, 0.0 /* kdz */, 0.0 /* kdphi */};
// ═══════════════════════════════════════════════════════════════════════════

// TAU_MAX disponible como hw_.limits.tau_max (cargado desde motorXM430W350T_params.yaml, default 1.2 N·m)
static constexpr double LAMBDA    = 0.01;    // amortiguamiento DLS (rango tipico: 0.01-0.05)

// ============================================================
// Nodo ROS 2
// ============================================================

class HWIOControlNode : public rclcpp::Node
{
public:
  explicit HWIOControlNode(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions())
  : Node("hw_io_control_node", opts), y0_initialized_(false)
  {
    // ── Parametros ──────────────────────────────────────────────────────────
    this->declare_parameter<std::string>("port_name",  "/dev/ttyUSB0");
    this->declare_parameter<double>     ("gain_scale",  1.0);
    this->declare_parameter<double>     ("t_run",       20.0);
    this->declare_parameter<int>        ("log_id",      1);

    this->declare_parameter<double>("ab_alpha", 0.2);
    this->declare_parameter<double>("ab_beta",  0.02);
    this->declare_parameter<double>("loop_rate_hz", 200.0);

    // Modelo del motor, calibracion y limites (config/motorXM430W350T_params.yaml)
    hw_ = omx::declareHwConfig(*this);

    port_name_  = this->get_parameter("port_name").as_string();
    gain_scale_ = this->get_parameter("gain_scale").as_double();
    t_run_      = this->get_parameter("t_run").as_double();
    const int log_id = this->get_parameter("log_id").as_int();

    ab_alpha_ = get_parameter("ab_alpha").as_double();
    ab_beta_  = get_parameter("ab_beta").as_double();
    loop_rate_hz_ = std::min(std::max(get_parameter("loop_rate_hz").as_double(), 50.0), 400.0);
    Ts_ = 1.0 / loop_rate_hz_;

    RCLCPP_INFO(get_logger(), "puerto=%s  gain=%.2f  dur=%.1fs  id=%d",
      port_name_.c_str(), gain_scale_, t_run_, log_id);
    RCLCPP_INFO(get_logger(),
      "Kp_y=[%.1f %.1f %.1f %.1f]  Kd_y=[%.1f %.1f %.1f %.1f]  lambda=%.3f  tau_max=%.2f",
      KP_Y[0], KP_Y[1], KP_Y[2], KP_Y[3],
      KD_Y[0], KD_Y[1], KD_Y[2], KD_Y[3],
      LAMBDA, hw_.limits.tau_max);
    RCLCPP_INFO(get_logger(),
      "motor alpha=[%.1f %.1f %.1f %.1f]  Fv=[%.2f %.2f %.2f %.2f]  epsilon=%.3f",
      hw_.motor.alpha(0), hw_.motor.alpha(1), hw_.motor.alpha(2), hw_.motor.alpha(3),
      hw_.motor.Fv(0), hw_.motor.Fv(1), hw_.motor.Fv(2), hw_.motor.Fv(3), hw_.motor.epsilon);
    RCLCPP_INFO(get_logger(),
      "Filtro alpha-beta: alpha=%.3f  beta=%.4f  |  Fc_scale=%.2f  |  lazo=%.0f Hz",
      ab_alpha_, ab_beta_, hw_.motor.fc_scale, loop_rate_hz_);

    // ── Pinocchio ────────────────────────────────────────────────────────────
    const std::string urdf = std::string(PACKAGE_URDF_DIR) + "/open_manipulator_x.urdf";
    try {
      model_.load(urdf, EFF_FRAME_NAME);   // lanza si falta el URDF o el frame del efector
    } catch (const std::exception& e) {
      RCLCPP_FATAL(get_logger(), "Pinocchio URDF: %s", e.what());
      throw;
    }
    RCLCPP_INFO(get_logger(), "Pinocchio: nv=%d  frame='%s'", model_.nv(), EFF_FRAME_NAME);

    // ── CSV ──────────────────────────────────────────────────────────────────
    csv_path_ = std::string(PACKAGE_DATA_DIR) + "/lab4/real/act2/hw_io_data_"
                + std::to_string(log_id) + ".csv";
    omx::openCsv(csv_, csv_path_,
      "t,q1,q2,q3,q4,"
      "dq1,dq2,dq3,dq4,"
      "dq1_filt,dq2_filt,dq3_filt,dq4_filt,"
      "x,y,z,phi,x_des,y_des,z_des,phi_des,"
      "xdot,ydot,zdot,phidot,xdot_des,ydot_des,zdot_des,phidot_des,"
      "tau1,tau2,tau3,tau4,"
      "curr_cmd1,curr_cmd2,curr_cmd3,curr_cmd4,"
      "curr_meas1,curr_meas2,curr_meas3,curr_meas4,"
      "det_J4",
      get_logger());

    // ── Publisher de monitoreo ────────────────────────────────────────────────
    js_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/hw/joint_states", 10);

    // ── SDK ──────────────────────────────────────────────────────────────────
    if (!arm_.open(port_name_, hw_.calib, hw_.limits.current_limit_register)) {
      RCLCPP_FATAL(get_logger(), "Fallo hardware init. Abortando.");
      throw std::runtime_error("Hardware init failed");
    }

    // ── Timer de control (loop_rate_hz) ──────────────────────────────────────
    start_time_ = std::chrono::high_resolution_clock::now();
    const auto period = std::chrono::microseconds(
      static_cast<int64_t>(std::lround(1e6 / loop_rate_hz_)));
    timer_ = this->create_wall_timer(period, [this]() { tick(); });
    RCLCPP_INFO(get_logger(), "Control IO activo a %.0f Hz (Ts=%.1f ms). Ctrl+C para detener.",
      loop_rate_hz_, 1e3 * Ts_);
  }

  ~HWIOControlNode()
  {
    if (timer_) timer_->cancel();
    arm_.close();
    if (csv_.is_open()) { csv_.flush(); csv_.close(); }
    RCLCPP_INFO(get_logger(), "Nodo finalizado.");
  }

private:
  // ─────────────────────────────────────────────────────────────────────────
  //  Conversion torque -> corriente via modelo OLS identificado
  //    I_cmd = alpha*tau + Fv*dq + fc_scale*Fc*tanh(dq_des/epsilon) + I_offset
  //  Fv usa la velocidad medida (termino suave); Fc usa la velocidad articular
  //  DESEADA (mapeo DLS de ydot_des): senal sin ruido → sin chattering, y
  //  aporta el empuje de despegue cuando la referencia arranca.
  // ─────────────────────────────────────────────────────────────────────────

  // ─────────────────────────────────────────────────────────────────────────
  //  Ley de control IO
  // ─────────────────────────────────────────────────────────────────────────

  struct IOOut {
    Vec4            tau;
    Vec4            dq_des;   // velocidad articular deseada (DLS de ydot_des)
    Eigen::Vector4d y_actual, ydot_actual, ydot_des;
    double          det_J4;
  };

  IOOut compute_io_control(const Vec4& q, const Vec4& dq, const CartRef& ref)
  {

    // Cinematica directa y Jacobiano 6×nv
    const auto ee = model_.eeKinematics(q, dq);   // p, J lineal y J̇·q̇ del efector

    const Eigen::Vector3d p   = ee.p;
    const double          phi = q[1] + q[2] + q[3];

    // Jacobiano de tarea 4×4: filas {vx, vy, vz, dphi/dq} — columnas {j1..j4}
    Eigen::Matrix4d J4;
    J4.topRows<3>() = ee.J;
    J4.row(3) << 0.0, 1.0, 1.0, 1.0;

    // Velocidad cartesiana actual: ydot = J4 * dq
    const Eigen::Vector4d ydot = J4 * dq;

    // Velocidad articular deseada (infraestructura, para la compensacion de
    // Coulomb en torque_to_current): mapeo DLS de ydot_des. La SECCION 3
    // construye su propia matriz A segun las instrucciones.
    const Eigen::Matrix4d A_fric = J4*J4.transpose()
                                 + LAMBDA*LAMBDA*Eigen::Matrix4d::Identity();
    const Vec4 dq_des_fric = J4.transpose() * A_fric.ldlt().solve(ref.ydot);

    // Termino de bias Jdot*dq (FK con qddot=0)
    Eigen::Vector4d jdqd;
    jdqd << ee.dJdq, 0.0;

    // Dinamica M(q) y nle(q, dq)

    const Eigen::Matrix4d M4   = model_.massMatrix(q);
    const Eigen::Vector4d nle4 = model_.nonlinearEffects(q, dq);

    // Salida de tarea actual
    const Eigen::Vector4d y_actual{p[0], p[1], p[2], phi};

    // ═══════════════════════════════════════════════════════════════════════
    //  SECCION 3 — LEY DE CONTROL (IO Linearization)
    //
    //  Disponible:
    //    y_actual (4×1)          — salida de tarea actual   [x, y, z, phi]
    //    ydot     (4×1)          — velocidad de la salida actual
    //    ref.y, ref.ydot, ref.yddot — referencia y sus derivadas analiticas
    //    J4   (4×4)              — Jacobiano de tarea
    //    jdqd (4×1)              — termino Jdot*dq (bias de aceleracion)
    //    M4   (4×4)              — matriz de masa inercial  M(q)
    //    nle4 (4×1)              — efectos no lineales      nle(q, dq)
    //    KP_Y, KD_Y              — ganancias (definidas en Seccion 2)
    //    gain_scale_             — factor de escala de ganancias
    //    TAU_MAX                 — saturacion de torque
    //
    //  Ley de control:
    //    ey    = ref.y    - y_actual          (error de posicion)
    //    eydot = ref.ydot - ydot              (error de velocidad)
    //    v     = ref.yddot + Kp_y.*ey + Kd_y.*eydot  (entrada auxiliar)
    //    A     = J4*J4^T + lambda^2 * I       (matriz DLS)
    //    qddot = J4^T * A^-1 * (v - jdqd)    (pseudo-inversa DLS)
    //    tau   = M4 * qddot + nle4            (torque de control)
    //    tau_sat = clamp(tau, -TAU_MAX, TAU_MAX)
    // ═══════════════════════════════════════════════════════════════════════

    // -- COMPLETAR --
    (void)M4; (void)nle4; (void)jdqd;
    const Eigen::Vector4d tau_sat = Vec4::Zero();  // reemplazar con tau calculado
    // ════════════════

    IOOut out;
    out.tau         = tau_sat;
    out.dq_des      = dq_des_fric;
    out.y_actual    = y_actual;
    out.ydot_actual = ydot;
    out.ydot_des    = ref.ydot;
    out.det_J4      = J4.determinant();
    return out;
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Parada de emergencia
  // ─────────────────────────────────────────────────────────────────────────

  void emergency_stop(const std::string& reason)
  {
    RCLCPP_ERROR(get_logger(), "PARADA: %s", reason.c_str());
    timer_->cancel();
    arm_.close();
    if (csv_.is_open()) { csv_.flush(); csv_.close(); }
    rclcpp::shutdown();
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Callback del timer (loop_rate_hz)
  // ─────────────────────────────────────────────────────────────────────────

  void tick()
  {
    if (!arm_.isOpen()) return;

    const auto tick_t0 = std::chrono::steady_clock::now();
    const auto   tp = std::chrono::high_resolution_clock::now();
    const double t  = std::chrono::duration<double>(tp - start_time_).count();

    const double t_shutdown = t_run_ + RETURN_TIME_S + HOLD_TIME_S;
    if (t_run_ > 0.0 && t >= t_shutdown) {
      RCLCPP_INFO(get_logger(), "Retorno a y_inicial completado. Deteniendo (t=%.1f s).", t);
      timer_->cancel();
      arm_.close();
      if (csv_.is_open()) { csv_.flush(); csv_.close(); }
      rclcpp::shutdown();
      return;
    }

    // 1. Lectura de estado
    Vec4 q, dq;
    std::array<int16_t, NUM_JOINTS> cur_meas{};
    if (!arm_.read(q, dq, cur_meas)) {
      emergency_stop("SyncRead fallido");
      return;
    }

    // 2. Filtro alpha-beta: estimacion conjunta de posicion y velocidad
    //   prediccion: q_pred = q_hat + Ts·dq_hat,  dq_pred = dq_hat
    //   residuo:    r      = q_meas - q_pred
    //   correccion: q_hat  = q_pred + alpha·r,    dq_hat = dq_pred + (beta/Ts)·r
    if (!ab_initialized_) {
      q_hat_          = q;
      dq_hat_         = dq;
      ab_initialized_ = true;
    } else {
      const Vec4 q_pred  = q_hat_ + Ts_ * dq_hat_;
      const Vec4 dq_pred = dq_hat_;
      const Vec4 r       = q - q_pred;
      q_hat_  = q_pred  + ab_alpha_ * r;
      dq_hat_ = dq_pred + (ab_beta_ / Ts_) * r;
    }

    // 3. Capturar pose cartesiana inicial (primer tick)
    if (!y0_initialized_) {
      const Eigen::Vector3d p0 = model_.eePosition(q);
      const double phi0        = q[1] + q[2] + q[3];
      y0_ = Eigen::Vector4d{p0[0], p0[1], p0[2], phi0};
      y0_initialized_ = true;
      RCLCPP_INFO(get_logger(),
        "y_inicial=[%.3f %.3f %.3f %.3f]  Y_START=[%.3f %.3f %.3f %.3f]",
        y0_[0], y0_[1], y0_[2], y0_[3],
        Y_START[0], Y_START[1], Y_START[2], Y_START[3]);
    }

    // 4. Verificar limites articulares del estado real
    for (int i = 0; i < NUM_JOINTS; ++i) {
      if (q(i) < hw_.limits.joint_lower(i) || q(i) > hw_.limits.joint_upper(i)) {
        emergency_stop("Articulacion " + std::to_string(i+1) + " fuera de limites: "
                       + std::to_string(q(i)) + " rad");
        return;
      }
    }

    // 5. Referencia cartesiana segun fase (transicion inicial + trayectoria + retorno final)
    CartRef ref;
    if (t < T_TRANS) {
      ref = omx::quinticTransition(t, T_TRANS, y0_, CartRef{Y_START, Eigen::Vector4d::Zero(), Eigen::Vector4d::Zero()});
    } else if (t_run_ <= 0.0 || t < t_run_) {
      ref = cartesianTrajectory(t - T_TRANS);
    } else if (t < t_run_ + RETURN_TIME_S) {
      if (!return_logged_) {
        RCLCPP_INFO(get_logger(), "Iniciando retorno cartesiano a y_inicial (t=%.1fs, dur=%.1fs)",
          t, RETURN_TIME_S);
        return_logged_ = true;
      }
      // Retorno quintico desde el estado de la trayectoria en t_run (posicion,
      // velocidad y aceleracion continuas, como FL y SMC) hasta y0 en reposo
      ref = omx::quinticReturn(t - t_run_, RETURN_TIME_S, cartesianTrajectory(t_run_ - T_TRANS), y0_);
    } else {
      ref.y = y0_;
      ref.ydot.setZero();
      ref.yddot.setZero();
    }

    // 5b. Guarda de altura minima de la REFERENCIA (placa base metalica).
    //     Cerca de la pose inicial (transicion de entrada, retorno final o
    //     pausa en reposo) el brazo puede estar bajo (y0_[2] < Z_MIN_FLOOR):
    //     solo se exige no descender mas de 3 cm bajo la pose inicial.
    const bool near_home = (t < T_TRANS) || (t_run_ > 0.0 && t >= t_run_);
    const double z_floor = near_home
      ? std::min(Z_MIN_FLOOR, y0_[2] - 0.03)
      : Z_MIN_FLOOR;
    if (ref.y[2] < z_floor) {
      emergency_stop("Referencia z bajo altura minima: "
                     + std::to_string(ref.y[2]) + " m (piso " + std::to_string(z_floor) + ")");
      return;
    }

    // 6. Ley de control IO — usa dq_hat_ para reducir chattering
    const IOOut ctrl = compute_io_control(q, dq_hat_, ref);

    // 6b. Guarda de altura minima del EFECTOR medido (FK de Pinocchio)
    if (ctrl.y_actual[2] < z_floor) {
      emergency_stop("Efector bajo altura minima: z="
                     + std::to_string(ctrl.y_actual[2]) + " m (piso " + std::to_string(z_floor) + ")");
      return;
    }

    // 7. Conversion torque -> corriente (saturacion defensiva a +/-tau_max;
    //    Fc con la velocidad articular deseada)
    const Vec4 tau = ctrl.tau.cwiseMin(hw_.limits.tau_max).cwiseMax(-hw_.limits.tau_max);
    const auto cur_cmd = omx::torqueToCurrent(hw_, tau, dq_hat_, ctrl.dq_des);

    // 8. Escribir corriente
    if (!arm_.writeCurrents(cur_cmd)) {
      emergency_stop("SyncWrite fallido");
      return;
    }

    // 9. Verificar corriente medida
    for (int i = 0; i < NUM_JOINTS; ++i) {
      if (std::abs(cur_meas[i]) > hw_.limits.current_measured_peak) {
        emergency_stop("Corriente insegura J" + std::to_string(i+1)
                       + ": " + std::to_string(cur_meas[i]) + " ticks");
        return;
      }
    }

    // 10. Publicar JointState de monitoreo
    {
      sensor_msgs::msg::JointState js;
      js.header.stamp = this->now();
      js.name         = {"joint1", "joint2", "joint3", "joint4"};
      js.position     = {q(0), q(1), q(2), q(3)};
      js.velocity     = {dq(0), dq(1), dq(2), dq(3)};
      js.effort = {
        static_cast<double>(cur_meas[0]) * omx::TORQUE_PER_CURRENT_TICK,
        static_cast<double>(cur_meas[1]) * omx::TORQUE_PER_CURRENT_TICK,
        static_cast<double>(cur_meas[2]) * omx::TORQUE_PER_CURRENT_TICK,
        static_cast<double>(cur_meas[3]) * omx::TORQUE_PER_CURRENT_TICK
      };
      js_pub_->publish(js);
    }

    // 11. CSV (excluye el retorno final a y_inicial y la pausa en reposo,
    //     asi el CSV solo cubre transicion inicial + trayectoria periodica)
    const bool in_return_or_hold = (t_run_ > 0.0 && t >= t_run_);
    if (csv_.is_open() && !in_return_or_hold) {
      csv_ << std::fixed << std::setprecision(6)
           << t
           << ',' << q(0) << ',' << q(1) << ',' << q(2) << ',' << q(3)
           << ',' << dq(0) << ',' << dq(1) << ',' << dq(2) << ',' << dq(3)
           << ',' << dq_hat_(0) << ',' << dq_hat_(1) << ',' << dq_hat_(2) << ',' << dq_hat_(3)
           << ',' << ctrl.y_actual[0]  << ',' << ctrl.y_actual[1]
           << ',' << ctrl.y_actual[2]  << ',' << ctrl.y_actual[3]
           << ',' << ref.y[0]  << ',' << ref.y[1]  << ',' << ref.y[2]  << ',' << ref.y[3]
           << ',' << ctrl.ydot_actual[0] << ',' << ctrl.ydot_actual[1]
           << ',' << ctrl.ydot_actual[2] << ',' << ctrl.ydot_actual[3]
           << ',' << ctrl.ydot_des[0] << ',' << ctrl.ydot_des[1]
           << ',' << ctrl.ydot_des[2] << ',' << ctrl.ydot_des[3]
           << ',' << tau[0] << ',' << tau[1]
           << ',' << tau[2] << ',' << tau[3]
           << ',' << cur_cmd[0] << ',' << cur_cmd[1]
           << ',' << cur_cmd[2] << ',' << cur_cmd[3]
           << ',' << cur_meas[0] << ',' << cur_meas[1]
           << ',' << cur_meas[2] << ',' << cur_meas[3]
           << ',' << ctrl.det_J4
           << '\n';
    }

    // 12. Log periodico (~1 s)
    if (++log_cnt_ % static_cast<int>(std::lround(loop_rate_hz_)) == 0) {
      if (csv_.is_open()) csv_.flush();
      const char* phase = (t < T_TRANS) ? "TRANS" :
                          (t_run_ <= 0.0 || t < t_run_) ? "TRAJ " :
                          (t < t_run_ + RETURN_TIME_S)  ? "RET  " : "HOME ";
      RCLCPP_INFO(get_logger(),
        "[%s] t=%.2fs  y=[%.3f %.3f %.3f %.3f]  |ey|=%.4f  detJ=%.4f",
        phase, t,
        ctrl.y_actual[0], ctrl.y_actual[1], ctrl.y_actual[2], ctrl.y_actual[3],
        (ref.y - ctrl.y_actual).norm(), ctrl.det_J4);
    }

    // 13. Deteccion de overrun: si el ciclo (bus + control) no cabe en Ts,
    //     el lazo real corre mas lento de lo configurado → bajar loop_rate_hz
    //     o revisar el latency_timer del adaptador USB-serial.
    const double tick_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - tick_t0).count();
    if (tick_ms > 1e3 * Ts_) {
      if (++overrun_cnt_ % 50 == 1) {
        RCLCPP_WARN(get_logger(), "Overrun del lazo: tick=%.2f ms > Ts=%.1f ms (n=%d)",
          tick_ms, 1e3 * Ts_, overrun_cnt_);
      }
    }
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Miembros
  // ─────────────────────────────────────────────────────────────────────────

  omx::RobotModel model_;

  std::string port_name_;
  double gain_scale_, t_run_;
  omx::HwConfig hw_;   // modelo del motor, calibracion y limites (YAML del motor)

  double ab_alpha_, ab_beta_;
  double loop_rate_hz_{200.0};
  double Ts_{0.005};
  Vec4   q_hat_, dq_hat_;
  bool   ab_initialized_{false};

  omx::DynamixelArm arm_{this->get_logger()};

  bool y0_initialized_;
  bool return_logged_{false};
  Eigen::Vector4d y0_;

  std::chrono::high_resolution_clock::time_point start_time_;
  int log_cnt_{0};
  int overrun_cnt_{0};

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::ofstream csv_;
  std::string   csv_path_;
};

// ============================================================
// main
// ============================================================

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  const rclcpp::NodeOptions opts = omx::motorParamsNodeOptions(argc, argv, "hw_io_control_node");

  try {
    rclcpp::spin(std::make_shared<HWIOControlNode>(opts));
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("hw_io_control_node"), "Excepcion: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
