/*
 * hw_impedance_control_node.cpp
 * Control de impedancia articular — OpenMANIPULATOR-X hardware real.
 * Via Dynamixel SDK directo, sin ros2_control.
 *
 * Ley de control (por articulacion):
 *   tau = K.*(q_ref - q) + D.*(dq_ref - dq_hat) + gravity_scale.*G(q)   [N·m]
 *   I   = alpha*tau + fc_scale*Fc*tanh(dq_ref/eps)                      [ticks]
 *
 * El robot se comporta como un resorte-amortiguador virtual anclado en q_des:
 * al alejarlo manualmente, regresa automaticamente; el torque de retorno esta
 * acotado por tau_max (1.2 N·m) → compliant e intrinsecamente seguro.
 *
 * Secuencia de arranque (todo en Current Mode, sin cambio de modo del motor):
 *   1) t < soft_start_s          : rampa quintica 0→1 del torque con la
 *      referencia CONGELADA en la pose inicial (el brazo no se mueve; evita
 *      el escalon de corriente que rompe la stiction via backlash — leccion
 *      del nodo de gravity comp).
 *   2) t < soft_start_s + t_trans: transicion quintica q_inicial → q_des.
 *      La PROPIA ley de impedancia rastrea la referencia movil; el torque
 *      queda acotado por tau_max durante todo el trayecto (a diferencia del
 *      Position Mode del motor, cuyo PID interno empuja hasta Current Limit
 *      y ademas exigiria torque-off para conmutar de modo, dejando caer el
 *      brazo un instante).
 *   3) t >= soft_start_s+t_trans : impedancia pura en q_des (dq_ref = 0).
 *
 * NOTA friccion: Fc se alimenta SOLO con la velocidad DESEADA (limpia, y nula
 * al mantener la pose) → ayuda al despegue durante la transicion y desaparece
 * en regimen. Fv e I_offset NO se aplican: con velocidad medida son
 * realimentacion positiva / sesgo en semi-lazo-abierto (diagnostico
 * hw_gc_data_1/2 de gravity comp). La friccion residual del reductor suma
 * amortiguamiento natural; la zona muerta de llegada es ≈ Fc/K por junta.
 *
 * Parametros del MOTOR (alpha, Fc, eps) y config de hardware (zero_tick,
 * signos, limites): auto-cargados de config/motorXM430W350T_params.yaml y
 * FIJOS (el YAML tiene prioridad sobre el CLI).
 *
 * Parametros propios del nodo (ros2 run ... --ros-args -p nombre:=valor):
 *   q_des             [double[4]] [0.0, 0.0, 0.0, 0.0]  pose a mantener [rad]
 *   stiffness         [double[4]] [4.0, 5.0, 5.0, 2.5]  K [N·m/rad]
 *   damping           [double[4]] [0.35, 0.40, 0.28, 0.07] D [N·m·s/rad]
 *                      (≈ 2·0.7·sqrt(K·M_ii) en la pose home; la friccion
 *                       real agrega amortiguamiento extra)
 *   gravity_scale       [double]   1.0
 *   gravity_scale_joint [double[4]] [1.0, 0.85, 0.85, 1.0] (correccion del
 *                        G(q) del URDF identificada en gravity comp)
 *   soft_start_s      [double]   1.5    rampa inicial de torque
 *   t_trans           [double]   3.0    transicion quintica hacia q_des
 *   dq_max            [double]   4.8    parada de seguridad [rad/s]
 *   friction_fc_scale [double]   0.95   fraccion de Fc en feedforward
 *   ab_alpha          [double]   0.2    filtro α-β (posicion)
 *   ab_beta           [double]   0.02   filtro α-β (velocidad)
 *   loop_rate_hz      [double]   200.0  acotado a [50, 400]
 *   duration_s        [double]   0.0    (0 = sin limite de tiempo)
 *   log_id            [int]      1      CSV: hw_imp_data_<log_id>.csv
 *   port_name         [string]   "/dev/ttyUSB0"
 *
 * Ejemplos de uso (no requiere launch; el YAML se auto-carga):
 *   ros2 run open_manipulator_x_torque_control hw_impedance_control_node \
 *     --ros-args -p log_id:=1
 *   ros2 run open_manipulator_x_torque_control hw_impedance_control_node \
 *     --ros-args -p q_des:="[0.0, -0.3, 0.4, 0.5]" \
 *     -p stiffness:="[6.0, 8.0, 8.0, 3.0]" -p damping:="[0.4, 0.5, 0.35, 0.1]"
 *
 * Publisher:  /hw/joint_states (sensor_msgs/JointState) — monitoreo
 * CSV output: data/impedance/hw_imp_data_<log_id>.csv
 *
 * ADVERTENCIA: No ejecutar junto a otro proceso que acceda a /dev/ttyUSB0
 * (ros2_control_node, dynamixel_hardware_interface, etc.).
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
#include <vector>

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

static constexpr double PI = 3.14159265358979323846;
static constexpr int    NUM_JOINTS = 4;

// Margen tolerado mas alla del limite articular antes de la parada de seguridad
static constexpr double LIMIT_MARGIN_RAD = 0.05;
// Margen minimo de q_des respecto a los limites articulares
static constexpr double QDES_MARGIN_RAD = 0.02;

using Vec4 = Eigen::Matrix<double, NUM_JOINTS, 1>;

// ============================================================
// Nodo ROS 2
// ============================================================

class HWImpedanceControlNode : public rclcpp::Node
{
public:
  explicit HWImpedanceControlNode(const rclcpp::NodeOptions & opts = rclcpp::NodeOptions())
  : Node("hw_impedance_control_node", opts),
    q_initial_captured_(false)
  {
    // ── Parametros propios del nodo ──────────────────────────────────────────
    this->declare_parameter<std::string>("port_name",         "/dev/ttyUSB0");
    this->declare_parameter<double>     ("gravity_scale",      1.0);
    this->declare_parameter<double>     ("soft_start_s",       1.5);
    this->declare_parameter<double>     ("t_trans",            3.0);
    this->declare_parameter<double>     ("dq_max",             4.8);
    this->declare_parameter<double>     ("ab_alpha",           0.2);
    this->declare_parameter<double>     ("ab_beta",            0.02);
    this->declare_parameter<double>     ("loop_rate_hz",       200.0);
    this->declare_parameter<double>     ("duration_s",         0.0);
    this->declare_parameter<int>        ("log_id",             1);

    using dvec = std::vector<double>;
    this->declare_parameter<dvec>("q_des",     dvec{0.0, 0.0, 0.0, 0.0});
    this->declare_parameter<dvec>("stiffness", dvec{4.0, 5.0, 5.0, 2.5});
    this->declare_parameter<dvec>("damping",   dvec{0.35, 0.40, 0.28, 0.07});
    this->declare_parameter<dvec>("gravity_scale_joint", dvec{1.0, 0.85, 0.85, 1.0});

    // Modelo del motor, calibracion y limites (config/motorXM430W350T_params.yaml)
    hw_ = omx::declareHwConfig(*this);

    port_name_     = this->get_parameter("port_name").as_string();
    gravity_scale_ = this->get_parameter("gravity_scale").as_double();
    soft_start_s_  = this->get_parameter("soft_start_s").as_double();
    t_trans_       = this->get_parameter("t_trans").as_double();
    dq_max_        = this->get_parameter("dq_max").as_double();
    ab_alpha_      = this->get_parameter("ab_alpha").as_double();
    ab_beta_       = this->get_parameter("ab_beta").as_double();
    loop_rate_hz_  = std::min(std::max(this->get_parameter("loop_rate_hz").as_double(), 50.0), 400.0);
    Ts_            = 1.0 / loop_rate_hz_;
    duration_s_    = this->get_parameter("duration_s").as_double();
    const int log_id = this->get_parameter("log_id").as_int();

    auto load_vec4 = [this](const std::string & name) {
      const auto v = this->get_parameter(name).as_double_array();
      Vec4 out; for (int i = 0; i < NUM_JOINTS; ++i) out(i) = v[i]; return out;
    };
    q_des_        = load_vec4("q_des");
    K_            = load_vec4("stiffness");
    D_            = load_vec4("damping");
    gscale_joint_ = load_vec4("gravity_scale_joint");

    // Validaciones: q_des dentro de limites, ganancias no negativas
    for (int i = 0; i < NUM_JOINTS; ++i) {
      if (q_des_(i) < hw_.limits.joint_lower(i) + QDES_MARGIN_RAD ||
          q_des_(i) > hw_.limits.joint_upper(i) - QDES_MARGIN_RAD) {
        RCLCPP_FATAL(get_logger(),
          "q_des[%d]=%.3f fuera de limites [%.3f, %.3f] (margen %.2f)",
          i, q_des_(i), hw_.limits.joint_lower(i), hw_.limits.joint_upper(i), QDES_MARGIN_RAD);
        throw std::runtime_error("q_des fuera de limites articulares");
      }
      if (K_(i) < 0.0 || D_(i) < 0.0) {
        RCLCPP_FATAL(get_logger(), "stiffness/damping[%d] negativo", i);
        throw std::runtime_error("Ganancias de impedancia invalidas");
      }
    }

    RCLCPP_INFO(get_logger(),
      "q_des=[%.3f %.3f %.3f %.3f]  K=[%.1f %.1f %.1f %.1f]  D=[%.2f %.2f %.2f %.2f]",
      q_des_(0), q_des_(1), q_des_(2), q_des_(3),
      K_(0), K_(1), K_(2), K_(3), D_(0), D_(1), D_(2), D_(3));
    RCLCPP_INFO(get_logger(),
      "g_scale=%.2f  g_joint=[%.2f %.2f %.2f %.2f]  rampa=%.1fs  trans=%.1fs  "
      "dq_max=%.1f  lazo=%.0f Hz  tau_max=%.2f",
      gravity_scale_, gscale_joint_(0), gscale_joint_(1), gscale_joint_(2), gscale_joint_(3),
      soft_start_s_, t_trans_, dq_max_, loop_rate_hz_, hw_.limits.tau_max);

    // ── Modelo dinamico (Pinocchio, via omx::RobotModel) ─────────────────────
    const std::string urdf = std::string(PACKAGE_URDF_DIR) + "/open_manipulator_x.urdf";
    try {
      model_.load(urdf);
    } catch (const std::exception & e) {
      RCLCPP_FATAL(get_logger(), "Pinocchio URDF: %s", e.what());
      throw;
    }
    RCLCPP_INFO(get_logger(), "Pinocchio: nv=%d", model_.nv());

    // ── CSV ──────────────────────────────────────────────────────────────────
    const std::string csv_dir = std::string(PACKAGE_DATA_DIR) + "/impedance";
    csv_path_ = csv_dir + "/hw_imp_data_" + std::to_string(log_id) + ".csv";
    omx::openCsv(csv_, csv_path_,
      "t,q1,q2,q3,q4,dq1,dq2,dq3,dq4,dq1_filt,dq2_filt,dq3_filt,dq4_filt,"
      "q1_ref,q2_ref,q3_ref,q4_ref,dq1_ref,dq2_ref,dq3_ref,dq4_ref,"
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
    RCLCPP_INFO(get_logger(), "Control activo a %.0f Hz. Ctrl+C para detener.", loop_rate_hz_);
  }

  ~HWImpedanceControlNode()
  {
    if (timer_) timer_->cancel();
    arm_.close();
    if (csv_.is_open()) { csv_.flush(); csv_.close(); }
    RCLCPP_INFO(get_logger(), "Nodo finalizado.");
  }

private:
  // ─────────────────────────────────────────────────────────────────────────
  //  Gravedad del modelo con correccion por junta:
  //  G_i(q) * gravity_scale * gravity_scale_joint[i]
  // ─────────────────────────────────────────────────────────────────────────

  Vec4 compute_gravity(const Vec4 & q)
  {
    const Vec4 g4 = model_.gravity(q);
    return gravity_scale_ * gscale_joint_.cwiseProduct(g4);
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Conversion torque → corriente:  I = alpha·tau + fc_scale·Fc·tanh(dq_ref/eps)
  //  Fc SOLO con velocidad deseada (nula en regimen → jamas realimenta el
  //  movimiento impuesto por el usuario). Sin Fv ni I_offset (ver cabecera).
  // ─────────────────────────────────────────────────────────────────────────

  omx::Currents torque_to_current(const Vec4 & tau, const Vec4 & dq_ref)
  {
    omx::Currents cmd{};
    for (int i = 0; i < NUM_JOINTS; ++i) {
      const double I = hw_.motor.alpha(i) * tau(i)
                     + hw_.motor.fc_scale * hw_.motor.Fc(i) * std::tanh(dq_ref(i) / hw_.motor.epsilon);
      cmd[i] = omx::clampCurrent(hw_.calib.current_sign(i) * hw_.calib.encoder_sign(i) * I,
                                 hw_.limits.current_cmd_limit[i]);
    }
    return cmd;
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Parada de emergencia
  // ─────────────────────────────────────────────────────────────────────────

  void emergency_stop(const std::string & reason)
  {
    RCLCPP_ERROR(get_logger(), "PARADA DE EMERGENCIA: %s", reason.c_str());
    timer_->cancel();
    arm_.close();
    if (csv_.is_open()) { csv_.flush(); csv_.close(); }
    rclcpp::shutdown();
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Callback del timer
  // ─────────────────────────────────────────────────────────────────────────

  void tick()
  {
    if (!arm_.isOpen()) return;

    const auto tick_t0 = std::chrono::steady_clock::now();
    const auto tp = std::chrono::high_resolution_clock::now();
    const double t = std::chrono::duration<double>(tp - start_time_).count();

    if (duration_s_ > 0.0 && t >= duration_s_) {
      RCLCPP_INFO(get_logger(), "Duracion completada (%.1f s).", duration_s_);
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

    // 1b. Filtro α-β: estimacion conjunta de posicion y velocidad
    if (!ab_initialized_) {
      q_hat_  = q;
      dq_hat_ = dq;
      ab_initialized_ = true;
    } else {
      const Vec4 q_pred  = q_hat_ + Ts_ * dq_hat_;
      const Vec4 dq_pred = dq_hat_;
      const Vec4 r       = q - q_pred;
      q_hat_  = q_pred  + ab_alpha_ * r;
      dq_hat_ = dq_pred + (ab_beta_ / Ts_) * r;
    }

    // 2. Guardas de seguridad: limites articulares y velocidad excesiva
    for (int i = 0; i < NUM_JOINTS; ++i) {
      if (q(i) < hw_.limits.joint_lower(i) - LIMIT_MARGIN_RAD ||
          q(i) > hw_.limits.joint_upper(i) + LIMIT_MARGIN_RAD) {
        emergency_stop("J" + std::to_string(i + 1) + " fuera de limite articular: q="
          + std::to_string(q(i)) + " rad");
        return;
      }
      if (std::abs(dq(i)) > dq_max_) {
        emergency_stop("Velocidad excesiva en J" + std::to_string(i + 1)
          + ": dq=" + std::to_string(dq(i)) + " rad/s");
        return;
      }
    }

    // 3. Captura de la pose inicial (primer tick)
    if (!q_initial_captured_) {
      q_initial_ = q;
      q_initial_captured_ = true;
      RCLCPP_INFO(get_logger(),
        "q_inicial=[%.3f %.3f %.3f %.3f] rad. Rampa %.1f s + transicion %.1f s hacia q_des.",
        q(0), q(1), q(2), q(3), soft_start_s_, t_trans_);
    }

    // 4. Referencia por fases: rampa (congelada en q_inicial) → quintica → q_des
    Vec4 q_ref, dq_ref;
    double ramp = 1.0;
    if (soft_start_s_ > 0.0 && t < soft_start_s_) {
      // Fase 1: la referencia se mantiene en q_inicial y el torque sube 0→1
      ramp   = omx::quinticProfile(t / soft_start_s_);
      q_ref  = q_initial_;
      dq_ref = Vec4::Zero();
    } else if (t < soft_start_s_ + t_trans_ && t_trans_ > 0.0) {
      // Fase 2: transicion quintica q_inicial → q_des
      if (!trans_announced_) {
        trans_announced_ = true;
        RCLCPP_INFO(get_logger(), "Rampa completada: transicion hacia q_des...");
      }
      const omx::Reference r = omx::quinticTransition(t - soft_start_s_, t_trans_, q_initial_,
                                                      omx::Reference{q_des_, Vec4::Zero(), Vec4::Zero()});
      q_ref  = r.q;
      dq_ref = r.dq;
    } else {
      // Fase 3: impedancia en q_des
      if (!hold_announced_) {
        hold_announced_ = true;
        RCLCPP_INFO(get_logger(),
          "Impedancia activa en q_des=[%.3f %.3f %.3f %.3f]. Puede empujar el robot.",
          q_des_(0), q_des_(1), q_des_(2), q_des_(3));
      }
      q_ref  = q_des_;
      dq_ref = Vec4::Zero();
    }

    // 5. Ley de impedancia: resorte-amortiguador virtual + compensacion de gravedad
    const Vec4 tau_unsat = ramp * (K_.cwiseProduct(q_ref - q)
                                 + D_.cwiseProduct(dq_ref - dq_hat_)
                                 + compute_gravity(q));
    const Vec4 tau = tau_unsat.cwiseMin(hw_.limits.tau_max).cwiseMax(-hw_.limits.tau_max);
    const auto cur_cmd = torque_to_current(tau, dq_ref);

    // 6. Escribir corriente
    if (!arm_.writeCurrents(cur_cmd)) {
      emergency_stop("SyncWrite fallido");
      return;
    }

    // 7. Verificar corriente medida (seguridad)
    for (int i = 0; i < NUM_JOINTS; ++i) {
      if (std::abs(cur_meas[i]) > hw_.limits.current_measured_peak) {
        emergency_stop("Corriente medida insegura en J" + std::to_string(i + 1)
          + ": " + std::to_string(cur_meas[i]) + " ticks");
        return;
      }
    }

    // 8. Publicar JointState de monitoreo
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

    // 9. CSV
    if (csv_.is_open()) {
      csv_ << std::fixed << std::setprecision(6) << t
           << ',' << q(0)       << ',' << q(1)       << ',' << q(2)       << ',' << q(3)
           << ',' << dq(0)      << ',' << dq(1)      << ',' << dq(2)      << ',' << dq(3)
           << ',' << dq_hat_(0) << ',' << dq_hat_(1) << ',' << dq_hat_(2) << ',' << dq_hat_(3)
           << ',' << q_ref(0)   << ',' << q_ref(1)   << ',' << q_ref(2)   << ',' << q_ref(3)
           << ',' << dq_ref(0)  << ',' << dq_ref(1)  << ',' << dq_ref(2)  << ',' << dq_ref(3)
           << ',' << tau(0)     << ',' << tau(1)     << ',' << tau(2)     << ',' << tau(3)
           << ',' << cur_cmd[0] << ',' << cur_cmd[1] << ',' << cur_cmd[2] << ',' << cur_cmd[3]
           << ',' << cur_meas[0]<< ',' << cur_meas[1]<< ',' << cur_meas[2]<< ',' << cur_meas[3]
           << '\n';
    }

    // 10. Log periodico por consola (~1 s)
    if (++log_cnt_ % static_cast<int>(std::lround(loop_rate_hz_)) == 0) {
      if (csv_.is_open()) csv_.flush();
      RCLCPP_INFO(get_logger(),
        "t=%.1fs  q=[%.3f %.3f %.3f %.3f]  |e|=%.4f  i=[%d %d %d %d]",
        t, q(0), q(1), q(2), q(3), (q - q_ref).norm(),
        cur_cmd[0], cur_cmd[1], cur_cmd[2], cur_cmd[3]);
    }

    // 11. Deteccion de overrun del lazo
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
  Vec4   q_des_, K_, D_;
  double gravity_scale_;
  Vec4   gscale_joint_;
  double soft_start_s_, t_trans_;
  double dq_max_;
  double duration_s_;

  // Modelo del motor, calibracion y limites (config/motorXM430W350T_params.yaml)
  omx::HwConfig hw_;

  double ab_alpha_, ab_beta_;
  double loop_rate_hz_{200.0};
  double Ts_{0.005};
  Vec4   q_hat_, dq_hat_;
  bool   ab_initialized_{false};

  omx::DynamixelArm arm_{this->get_logger()};

  bool q_initial_captured_;
  bool trans_announced_{false};
  bool hold_announced_{false};
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

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // Auto-carga config/motorXM430W350T_params.yaml; los -p de la línea de comandos
  // (q_des, stiffness, damping, gravity_scale, gravity_scale_joint, soft_start_s,
  // t_trans, dq_max, log_id, ...) lo sobrescriben → no requiere launch.
  const rclcpp::NodeOptions opts = omx::motorParamsNodeOptions(argc, argv, "hw_impedance_control_node");

  try {
    rclcpp::spin(std::make_shared<HWImpedanceControlNode>(opts));
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("hw_impedance_control_node"),
      "Excepcion: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
