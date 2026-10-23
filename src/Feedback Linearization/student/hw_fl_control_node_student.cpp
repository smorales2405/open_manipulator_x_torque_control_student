/*
 * hw_fl_control_node_student.cpp
 * Plantilla: Controlador Feedback Linearization — OpenMANIPULATOR-X hardware real
 * via Dynamixel SDK directo.
 *
 * Ley de control a implementar:
 *   tau = M(q) * v + nle(q, dq)
 *   v   = ddq_des + Kd*(dq_des - dq) + Kp*(q_des - q)
 *
 * Completar las tres secciones marcadas:
 *   SECCION 1 — Trayectoria deseada articular  (desiredTrajectory)
 *   SECCION 2 — Ganancias del controlador      (KP, KD globales)
 *   SECCION 3 — Ley de control FL              (v, tau en compute_torque)
 *
 * Infraestructura proporcionada (NO modificar):
 *   - Inicializacion Dynamixel SDK: apertura del puerto, modos, SyncRead/SyncWrite
 *   - Lectura de estado (q, dq, corriente medida) en UN solo SyncRead del bloque
 *     contiguo 126-135 (corriente+velocidad+posicion), a loop_rate_hz
 *   - Filtro alpha-beta de posicion/velocidad articular (ab_alpha, ab_beta)
 *   - Calculo de M(q) y nle(q,dq) mediante Pinocchio (en compute_torque)
 *   - Conversion torque -> corriente via modelo OLS identificado
 *     (motorXM430W350T_params.yaml); el termino de Coulomb usa la velocidad
 *     DESEADA (senal sin ruido: sin chattering y con empuje de despegue)
 *   - Saturacion del torque a +/-tau_max antes de enviar corriente
 *   - Transicion quintica de 3 s desde la posicion inicial hasta el inicio de la trayectoria
 *   - Retorno quintico al reposo en q_inicial al completar t_run + pausa en
 *     reposo antes de cortar corriente (evita que el brazo caiga por
 *     gravedad al recibir torque cero de golpe en medio de la trayectoria)
 *   - Verificacion de limites articulares y de corriente medida (parada de emergencia)
 *   - Deteccion de overruns del lazo de control
 *   - Publicador /hw/joint_states y escritura de CSV (el retorno final y la
 *     pausa en reposo no se registran en el CSV)
 *
 * Parametros ROS 2 (--ros-args -p nombre:=valor):
 *   port_name              [string]   "/dev/ttyUSB0"
 *   gain_scale             [double]   1.0    (escala lineal de Kp; Kd escala con sqrt)
 *   t_run                  [double]   20.0   (duracion; 0 = sin limite)
 *   log_id                 [int]      1
 *   ab_alpha               [double]   0.2    (filtro alpha-beta: ganancia de posicion)
 *   ab_beta                [double]   0.02   (filtro alpha-beta: ganancia de velocidad)
 *   friction_fc_scale      [double]   0.95   (fraccion de Fc compensada)
 *   loop_rate_hz           [double]   200.0  (frecuencia del lazo [Hz], acotada a [50,400])
 *
 * Parametros del modelo identificado (config/motorXM430W350T_params.yaml):
 *   motor_alpha            [double[4]]   ticks/N·m
 *   motor_Fv               [double[4]]   ticks/(rad/s)
 *   motor_Fc               [double[4]]   ticks
 *   motor_I_offset         [double[4]]   ticks
 *   motor_epsilon_friction [double]      rad/s
 *
 * Ejemplo de ejecucion:
 *   ros2 run open_manipulator_x_torque_control hw_fl_control_node_student \
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

static constexpr double RAMP_TIME_S   = 3.0;   // duracion de la transicion inicial [s]
static constexpr double RETURN_TIME_S = 3.0;   // duracion del retorno quintico a q_inicial
static constexpr double HOLD_TIME_S   = 0.5;   // pausa en reposo antes de cortar corriente

using Vec4 = Eigen::Matrix<double, NUM_JOINTS, 1>;

// ============================================================
// Estructura de referencia articular
// ============================================================

using omx::Reference;   // {q, dq, ddq}

// ═══════════════════════════════════════════════════════════════════════════
//  SECCION 1 — TRAYECTORIA DESEADA ARTICULAR
//
//  Definir ref.q, ref.dq y ref.ddq como funciones del tiempo t
//
//  Parametros de entrada:
//    t   — tiempo transcurrido desde el inicio de la trayectoria [s]
//
//  Salida esperada:
//    ref.q   << q1_des(t), q2_des(t), q3_des(t), q4_des(t);
//    ref.dq  << dq1_des(t),    dq2_des(t),    dq3_des(t),    dq4_des(t);
//    ref.ddq << ddq1_des(t),   ddq2_des(t),   ddq3_des(t),   ddq4_des(t);
//
//  Nota: esta funcion se llama para t >= RAMP_TIME_S.
//        Para t < RAMP_TIME_S se aplica la transicion quintica
//        automaticamente (ver quinticTransition — no modificar).
// ═══════════════════════════════════════════════════════════════════════════

static Reference desiredTrajectory(double t)
{
  Reference ref;

  // -- completar --
  (void)t;
  const double w = 1.0;
  (void)w;

  ref.q.setZero();
  ref.dq.setZero();
  ref.ddq.setZero();
  // ────────────────

  return ref;
}

// ═══════════════════════════════════════════════════════════════════════════

// ═══════════════════════════════════════════════════════════════════════════
//  SECCION 2 — GANANCIAS DEL CONTROLADOR
//
//  Definir KP y KD para cada articulacion: [j1, j2, j3, j4]
//    KP — ganancias proporcionales  [adim]
//    KD — ganancias derivativas     [adim]
//
//  gain_scale_ escala KP linealmente y KD con su raiz cuadrada
//
//  Sugerencia: con FL exacto, la dinamica del error por joint es
//    e'' + kd*e' + kp*e = 0   →   amortiguamiento zeta = kd/(2*sqrt(kp))
//  Mantener kd ≈ 1.4*sqrt(kp) (zeta ≈ 0.7) evita oscilaciones subamortiguadas.
//  Recordar ademas que el torque de feedback es M(q)*v: articulaciones con
//  inercia pequena (munecas) necesitan kp mayores para vencer la friccion.
// ═══════════════════════════════════════════════════════════════════════════
static const Eigen::Vector4d KP = {0.0 /* kp1 */, 0.0 /* kp2 */, 0.0 /* kp3 */, 0.0 /* kp4 */};   // COMPLETAR
static const Eigen::Vector4d KD = {0.0 /* kd1 */, 0.0 /* kd2 */, 0.0 /* kd3 */, 0.0 /* kd4 */};   // COMPLETAR
// TAU_MAX disponible como hw_.limits.tau_max (cargado desde motorXM430W350T_params.yaml, default 1.2 N·m)
// ═══════════════════════════════════════════════════════════════════════════

// ============================================================
// Nodo ROS 2
// ============================================================

class HWFLControlNode : public rclcpp::Node
{
public:
  explicit HWFLControlNode(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions())
  : Node("hw_fl_control_node", opts), q_initial_captured_(false)
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
      "motor alpha=[%.1f %.1f %.1f %.1f]  Fv=[%.2f %.2f %.2f %.2f]  epsilon=%.3f",
      hw_.motor.alpha(0), hw_.motor.alpha(1), hw_.motor.alpha(2), hw_.motor.alpha(3),
      hw_.motor.Fv(0), hw_.motor.Fv(1), hw_.motor.Fv(2), hw_.motor.Fv(3), hw_.motor.epsilon);
    RCLCPP_INFO(get_logger(),
      "Filtro alpha-beta: alpha=%.3f  beta=%.4f  |  Fc_scale=%.2f  |  lazo=%.0f Hz",
      ab_alpha_, ab_beta_, hw_.motor.fc_scale, loop_rate_hz_);

    // ── Pinocchio ────────────────────────────────────────────────────────────
    const std::string urdf = std::string(PACKAGE_URDF_DIR) + "/open_manipulator_x.urdf";
    try {
      model_.load(urdf);
    } catch (const std::exception& e) {
      RCLCPP_FATAL(get_logger(), "Pinocchio URDF: %s", e.what());
      throw;
    }
    RCLCPP_INFO(get_logger(), "Pinocchio: nv=%d", model_.nv());

    // ── CSV ──────────────────────────────────────────────────────────────────
    csv_path_ = std::string(PACKAGE_DATA_DIR) + "/lab4/real/act1/hw_fl_data_"
                + std::to_string(log_id) + ".csv";
    omx::openCsv(csv_, csv_path_,
      "t,q1,q2,q3,q4,dq1,dq2,dq3,dq4,dq1_filt,dq2_filt,dq3_filt,dq4_filt,"
      "q1_des,q2_des,q3_des,q4_des,dq1_des,dq2_des,dq3_des,dq4_des,"
      "tau1,tau2,tau3,tau4,"
      "curr_cmd1,curr_cmd2,curr_cmd3,curr_cmd4,"
      "curr_meas1,curr_meas2,curr_meas3,curr_meas4",
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
    RCLCPP_INFO(get_logger(), "Control activo a %.0f Hz (Ts=%.1f ms). Ctrl+C para detener.",
      loop_rate_hz_, 1e3 * Ts_);
  }

  ~HWFLControlNode()
  {
    if (timer_) timer_->cancel();
    arm_.close();
    if (csv_.is_open()) { csv_.flush(); csv_.close(); }
    RCLCPP_INFO(get_logger(), "Nodo finalizado.");
  }

private:
  // ─────────────────────────────────────────────────────────────────────────
  //  Ley de control FL
  // ─────────────────────────────────────────────────────────────────────────

  Vec4 compute_torque(const Vec4& q, const Vec4& dq, const Reference& ref)
  {
    // Aplicar escala de ganancia (gain_scale_)
    const Vec4 kp = KP * gain_scale_;
    const Vec4 kd = KD * std::sqrt(std::max(gain_scale_, 0.0));

    // Dinamica M(q) y nle(q, dq) via Pinocchio

    const Eigen::Matrix4d M4   = model_.massMatrix(q);
    const Eigen::Vector4d nle4 = model_.nonlinearEffects(q, dq);

    // ═══════════════════════════════════════════════════════════════════════
    //  SECCION 3 — LEY DE CONTROL (Feedback Linearization)
    //
    //  Disponible:
    //    q, dq                  — posicion y velocidad articular medidas
    //    ref.q, ref.dq, ref.ddq — referencia y sus derivadas analiticas
    //    M4   (4×4)             — matriz de masa inercial  M(q)
    //    nle4 (4×1)             — efectos no lineales      nle(q, dq)
    //    kp, kd                 — ganancias escaladas
    //
    //  Ley de control:
    //    e   = ref.q  - q              (error de posicion)
    //    de  = ref.dq - dq             (error de velocidad)
    //    v   = ref.ddq + kp.*e + kd.*de  (entrada auxiliar)
    //    tau = M4 * v + nle4           (torque de control)
    //
    //  Resultado: retornar tau (Vec4)
    // ═══════════════════════════════════════════════════════════════════════

    // -- completar --
    (void)M4; (void)nle4; (void)kp; (void)kd; (void)ref;
    return Vec4::Zero();  // reemplazar con tau calculado
    // ════════════════
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Conversion torque -> corriente via modelo OLS identificado
  //    I_cmd = alpha*tau + Fv*dq + fc_scale*Fc*tanh(dq_des/epsilon) + I_offset
  //  Fv usa la velocidad medida (termino suave); Fc usa la velocidad DESEADA:
  //  senal sin ruido → sin chattering, y aporta el empuje de despegue justo
  //  cuando la referencia arranca (con dq medida ≈ 0 el tanh muere pegado).
  // ─────────────────────────────────────────────────────────────────────────

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
      RCLCPP_INFO(get_logger(), "Retorno a q_inicial completado. Deteniendo (t=%.1f s).", t);
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

    // 3. Captura posicion inicial (primer tick)
    if (!q_initial_captured_) {
      q_initial_         = q;
      q_initial_captured_ = true;
      RCLCPP_INFO(get_logger(), "q_inicial=[%.3f %.3f %.3f %.3f] rad",
        q(0), q(1), q(2), q(3));
    }

    // 4. Referencia (transicion quintica + trayectoria + retorno final)
    Reference ref;
    if (t < RAMP_TIME_S) {
      ref = omx::quinticTransition(t, RAMP_TIME_S, q_initial_, desiredTrajectory(RAMP_TIME_S));
    } else if (t_run_ <= 0.0 || t < t_run_) {
      ref = desiredTrajectory(t);
    } else if (t < t_run_ + RETURN_TIME_S) {
      if (!return_logged_) {
        RCLCPP_INFO(get_logger(), "Iniciando retorno quintico a q_inicial (t=%.1fs, dur=%.1fs)",
          t, RETURN_TIME_S);
        return_logged_ = true;
      }
      ref = omx::quinticReturn(t - t_run_, RETURN_TIME_S, desiredTrajectory(t_run_), q_initial_);
    } else {
      ref.q = q_initial_;
      ref.dq.setZero();
      ref.ddq.setZero();
    }

    // 5. Verificar limites de la referencia
    for (int i = 0; i < NUM_JOINTS; ++i) {
      if (ref.q(i) < hw_.limits.joint_lower(i) + 0.02 || ref.q(i) > hw_.limits.joint_upper(i) - 0.02) {
        emergency_stop("Referencia fuera de limites articulares");
        return;
      }
    }

    // 6. Ley de control — usa dq_hat_ para reducir chattering; el torque se
    //    satura a +/-tau_max antes de convertirlo a corriente
    const Vec4 tau_unsat = compute_torque(q, dq_hat_, ref);
    const Vec4 tau = tau_unsat.cwiseMin(hw_.limits.tau_max).cwiseMax(-hw_.limits.tau_max);
    const auto cur_cmd = omx::torqueToCurrent(hw_, tau, dq_hat_, ref.dq);

    // 7. Escribir corriente
    if (!arm_.writeCurrents(cur_cmd)) {
      emergency_stop("SyncWrite fallido");
      return;
    }

    // 8. Verificar corriente medida (seguridad)
    for (int i = 0; i < NUM_JOINTS; ++i) {
      if (std::abs(cur_meas[i]) > hw_.limits.current_measured_peak) {
        emergency_stop("Corriente medida insegura en J" + std::to_string(i + 1)
                       + ": " + std::to_string(cur_meas[i]) + " ticks");
        return;
      }
    }

    // 9. Publicar JointState de monitoreo
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

    // 10. CSV (excluye el retorno final a q_inicial y la pausa en reposo,
    //     asi el CSV solo cubre la rampa inicial + trayectoria periodica)
    const bool in_return_or_hold = (t_run_ > 0.0 && t >= t_run_);
    if (csv_.is_open() && !in_return_or_hold) {
      csv_ << std::fixed << std::setprecision(6) << t
           << ',' << q(0)        << ',' << q(1)        << ',' << q(2)        << ',' << q(3)
           << ',' << dq(0)       << ',' << dq(1)       << ',' << dq(2)       << ',' << dq(3)
           << ',' << dq_hat_(0)  << ',' << dq_hat_(1)  << ',' << dq_hat_(2)  << ',' << dq_hat_(3)
           << ',' << ref.q(0)    << ',' << ref.q(1)    << ',' << ref.q(2)    << ',' << ref.q(3)
           << ',' << ref.dq(0)   << ',' << ref.dq(1)   << ',' << ref.dq(2)   << ',' << ref.dq(3)
           << ',' << tau(0)      << ',' << tau(1)       << ',' << tau(2)      << ',' << tau(3)
           << ',' << cur_cmd[0]  << ',' << cur_cmd[1]   << ',' << cur_cmd[2]  << ',' << cur_cmd[3]
           << ',' << cur_meas[0] << ',' << cur_meas[1]  << ',' << cur_meas[2] << ',' << cur_meas[3]
           << '\n';
    }

    // 11. Log periodico por consola (~1 s)
    if (++log_cnt_ % static_cast<int>(std::lround(loop_rate_hz_)) == 0) {
      if (csv_.is_open()) csv_.flush();
      RCLCPP_INFO(get_logger(),
        "t=%.2fs  q=[%.3f %.3f %.3f %.3f]  |e|=%.4f  i=[%d %d %d %d]",
        t, q(0), q(1), q(2), q(3), (q - ref.q).norm(),
        cur_cmd[0], cur_cmd[1], cur_cmd[2], cur_cmd[3]);
    }

    // 12. Deteccion de overrun: si el ciclo (bus + control) no cabe en Ts,
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

  bool q_initial_captured_;
  bool return_logged_{false};
  Vec4 q_initial_;
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

  const rclcpp::NodeOptions opts = omx::motorParamsNodeOptions(argc, argv, "hw_fl_control_node");

  try {
    rclcpp::spin(std::make_shared<HWFLControlNode>(opts));
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("hw_fl_control_node"), "Excepcion: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
