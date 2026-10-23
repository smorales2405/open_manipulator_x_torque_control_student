/*
 * hw_sinusoidal_torque_node.cpp
 * Diagnóstico: torques sinusoidales y/o control de posición por articulación.
 *
 * Cada articulación puede configurarse independientemente en:
 *   "current"  — envía τ_i(t') = A_i + B_i·sin(w_i·t')  [N·m → corriente via modelo OLS]
 *   "position" — sigue q_ref_i(t') = A_pos_i + B_pos_i·sin(w_pos_i·t') [rad] via Position Control Mode
 *   "velocity" — barrido de velocidades constantes (Velocity Control Mode) para
 *                identificar fricción: recorre vel_list[] (rad/s), rebotando dentro
 *                de A_pos_i ± vel_band. Con ddq=0 la corriente aísla la fricción.
 *
 * Secuencia de fases:
 *   SETTLING (t < t_settle):
 *     → TODOS los joints en Position Control Mode, van a pos_rad_i.
 *     → Permite que el robot alcance la configuración inicial antes de aplicar torques.
 *     → pos_rad: [0.0, 0.0, 0.0, 0.0]  lleva el robot a la posición cero de cada joint.
 *   RAMP (t_settle ≤ t_ctrl < t_ramp):
 *     → Joints en mode="current" cambian a Current Control Mode.
 *     → Rampa lineal del offset DC: τ_i = (t_ctrl/t_ramp)·A_i
 *   RUN (t_ctrl ≥ t_ramp):
 *     → Señal completa: τ_i = A_i + B_i·sin(w_i·t')
 *
 * Fin del ensayo (t ≥ duration_s) — retorno a la posición inicial, como en los
 * nodos hw_ de Feedback Linearization: cortar el torque en la pose del barrido
 * dejaría caer el brazo por gravedad.
 *   BRAKE  (BRAKE_TIME_S): joints en mode="velocity" → velocidad 0. Los joints
 *          en mode="current" pasan a Position Control Mode sosteniendo su pose.
 *   RETURN (≥ RETURN_TIME_S): TODOS los joints en Position Control Mode,
 *          quíntica desde su meta actual hasta q_inicial (posición leída en el
 *          primer ciclo, ANTES del settling). La duración se alarga si hace
 *          falta para que la velocidad pico no supere RETURN_VMAX.
 *   HOLD   (HOLD_TIME_S): pausa en q_inicial y recién entonces torque off.
 * El CSV solo registra SETTLING/RAMP/RUN (el retorno no se guarda, no altera el
 * análisis). duration_s ≤ 0 desactiva el fin: corre hasta Ctrl+C, SIN retorno
 * (igual que Ctrl+C o una parada de emergencia: el torque se corta en el acto).
 *
 * Ejecución:
 *   ros2 run open_manipulator_x_torque_control hw_sinusoidal_torque_node \
 *       --ros-args -p log_id:=3
 *
 * CSV de salida: data/diagnostics/sinusoidal/hw_sin_torque_<log_id>.csv
 *   t, q1..4, dq1..4, dq_ref1..4, ddq_ref1..4, tau_ref1..4, pos_ref1..4, curr_cmd1..4, curr_meas1..4
 */

#include <algorithm>
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

#include "omx/dynamixel_io.hpp"
#include "omx/motor_model.hpp"   // HwLimits
#include "omx/trajectory.hpp"     // quinticProfile

#ifndef PACKAGE_DATA_DIR
#define PACKAGE_DATA_DIR "."
#endif
#ifndef PACKAGE_CONFIG_DIR
#define PACKAGE_CONFIG_DIR "."
#endif

using namespace std::chrono_literals;

// ============================================================
// Constantes y conversiones de los servos: omx/dynamixel_io.hpp
// (tabla de control, unidades, modos de operacion). Calibracion y
// limites: omx::declareCalibration / omx::declareHwLimits.
// ============================================================

using namespace omx;

// Retorno final a la posición inicial (ver cabecera)
static constexpr double BRAKE_TIME_S  = 0.5;   // frenado de los joints en velocity
static constexpr double RETURN_TIME_S = 3.0;   // duración mínima del retorno quíntico
static constexpr double RETURN_VMAX   = 0.8;   // [rad/s] velocidad pico máx. del retorno
static constexpr double HOLD_TIME_S   = 0.5;   // pausa en q_inicial antes de cortar torque

// ============================================================
// Fases de operación
// ============================================================

enum class Phase { SETTLING, RAMP, RUN, BRAKE, RETURN, HOLD };

// ============================================================
// Utilidades de conversión
// ============================================================

// Convierte pos_rad [rad] al tick absoluto del servo
static int32_t radToTicks(const Calibration& calib, int joint_idx, double pos_rad)
{
  const int32_t t = static_cast<int32_t>(std::round(
    calib.joint_zero_tick[joint_idx]
    + calib.encoder_sign(joint_idx) * pos_rad / POS_UNIT_RAD));
  return std::clamp(t, 0, 4095);
}

static void positionToBytes(int32_t ticks, uint8_t p[4])
{
  const uint32_t u = static_cast<uint32_t>(std::clamp(ticks, 0, 4095));
  p[0] = DXL_LOBYTE(DXL_LOWORD(u));
  p[1] = DXL_HIBYTE(DXL_LOWORD(u));
  p[2] = DXL_LOBYTE(DXL_HIWORD(u));
  p[3] = DXL_HIBYTE(DXL_HIWORD(u));
}

// Goal Velocity es un entero con signo de 4 bytes (complemento a dos)
static void velocityToBytes(int32_t ticks, uint8_t p[4])
{
  const uint32_t u = static_cast<uint32_t>(ticks);
  p[0] = DXL_LOBYTE(DXL_LOWORD(u));
  p[1] = DXL_HIBYTE(DXL_LOWORD(u));
  p[2] = DXL_LOBYTE(DXL_HIWORD(u));
  p[3] = DXL_HIBYTE(DXL_HIWORD(u));
}

// ============================================================
// Nodo ROS 2
// ============================================================

class HWSinusoidalTorqueNode : public rclcpp::Node
{
public:
  explicit HWSinusoidalTorqueNode(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions())
  : Node("hw_sinusoidal_torque_node", opts), hw_active_(false), phase_(Phase::SETTLING)
  {
    // ── Parámetros ──────────────────────────────────────────────────────────
    this->declare_parameter<std::string>("port_name",               "/dev/ttyUSB0");
    this->declare_parameter<bool>       ("open_port",               false);
    this->declare_parameter<bool>       ("enable_torque",           false);
    this->declare_parameter<bool>       ("enable_current_commands", false);
    this->declare_parameter<int>        ("log_id",                  1);
    this->declare_parameter<double>     ("t_settle",                3.0);
    this->declare_parameter<double>     ("t_ramp",                  3.0);
    this->declare_parameter<double>     ("duration_s",              30.0);

    using svec = std::vector<std::string>;
    using dvec = std::vector<double>;
    this->declare_parameter<svec>("mode",    svec{"current","current","current","current"});
    this->declare_parameter<dvec>("A",       dvec{0.0, 0.0, 0.0, 0.0});
    this->declare_parameter<dvec>("B",       dvec{0.0, 0.0, 0.0, 0.0});
    this->declare_parameter<dvec>("w",       dvec{1.0, 1.0, 1.0, 1.0});
    this->declare_parameter<dvec>("A_pos",   dvec{0.0, 0.0, 0.0, 0.0});
    this->declare_parameter<dvec>("B_pos",   dvec{0.0, 0.0, 0.0, 0.0});
    this->declare_parameter<dvec>("w_pos",   dvec{0.0, 0.0, 0.0, 0.0});
    this->declare_parameter<dvec>("B_pos2",  dvec{0.0, 0.0, 0.0, 0.0});
    this->declare_parameter<dvec>("w_pos2",  dvec{0.0, 0.0, 0.0, 0.0});
    this->declare_parameter<dvec>("pos_rad", dvec{0.0, 0.0, 0.0, 0.0});
    this->declare_parameter<int> ("pos_profile_vel", 50);
    this->declare_parameter<int> ("pos_profile_acc", 20);

    // ── Barrido de velocidad constante (mode="velocity") ──────────────────────
    this->declare_parameter<dvec>  ("vel_list",         dvec{0.05,0.10,0.15,0.20,0.25,0.30});
    this->declare_parameter<double>("vel_seg_duration", 4.0);   // [s] por magnitud
    this->declare_parameter<double>("vel_band",         0.5);   // [rad] semirango alrededor de A_pos

    this->declare_parameter<dvec>  ("motor_alpha",            dvec{208.5,208.5,208.5,208.5});
    this->declare_parameter<dvec>  ("motor_Fv",               dvec{0.0,  0.0,  0.0,  0.0 });
    this->declare_parameter<dvec>  ("motor_Fc",               dvec{0.0,  0.0,  0.0,  0.0 });
    this->declare_parameter<dvec>  ("motor_I_offset",         dvec{0.0,  0.0,  0.0,  0.0 });
    this->declare_parameter<double>("motor_epsilon_friction", 0.05);

    // Calibracion y limites: defaults del YAML del motor (motorXM430W350T_params.yaml)
    calib_  = omx::declareCalibration(*this);
    limits_ = omx::declareHwLimits(*this);

    // ── Lectura ──────────────────────────────────────────────────────────────
    port_name_               = get_parameter("port_name").as_string();
    open_port_               = get_parameter("open_port").as_bool();
    enable_torque_           = get_parameter("enable_torque").as_bool();
    enable_current_commands_ = get_parameter("enable_current_commands").as_bool();
    const int log_id         = get_parameter("log_id").as_int();
    t_settle_                = get_parameter("t_settle").as_double();
    t_ramp_                  = get_parameter("t_ramp").as_double();
    duration_s_              = get_parameter("duration_s").as_double();

    joint_mode_ = get_parameter("mode").as_string_array();
    if ((int)joint_mode_.size() != NUM_JOINTS)
      throw std::runtime_error("Parámetro 'mode' debe tener 4 elementos");
    for (const auto& m : joint_mode_) {
      if (m != "current" && m != "position" && m != "velocity")
        throw std::runtime_error("Modo inválido '" + m + "'; usar 'current', 'position' o 'velocity'");
    }

    auto load_vec4 = [this](const std::string& name) {
      auto v = get_parameter(name).as_double_array();
      return Vec4(v[0], v[1], v[2], v[3]);
    };
    A_dc_           = load_vec4("A");
    B_amp_          = load_vec4("B");
    w_freq_         = load_vec4("w");
    A_pos_          = load_vec4("A_pos");
    B_pos_          = load_vec4("B_pos");
    w_pos_          = load_vec4("w_pos");
    B_pos2_         = load_vec4("B_pos2");
    w_pos2_         = load_vec4("w_pos2");
    pos_rad_        = load_vec4("pos_rad");
    pos_profile_vel_ = get_parameter("pos_profile_vel").as_int();
    pos_profile_acc_ = get_parameter("pos_profile_acc").as_int();
    goal_pos_        = pos_rad_;   // meta de posición vigente (settling)

    vel_list_         = get_parameter("vel_list").as_double_array();
    vel_seg_duration_ = get_parameter("vel_seg_duration").as_double();
    vel_band_         = get_parameter("vel_band").as_double();
    if (vel_list_.empty()) vel_list_ = {0.1};
    vel_dir_.fill(+1.0);

    // Barrido de velocidad: verificar que duration_s alcance para recorrer
    // TODO el vel_list (si no, el barrido se trunca y la identificación de
    // fricción pierde las velocidades altas).
    const bool any_vel = std::any_of(joint_mode_.begin(), joint_mode_.end(),
                                     [](const std::string& m) { return m == "velocity"; });
    if (any_vel && duration_s_ > 0.0) {
      const double t_need = t_settle_
        + static_cast<double>(vel_list_.size()) * vel_seg_duration_;
      if (duration_s_ < t_need) {
        RCLCPP_WARN(get_logger(),
          "duration_s=%.1f s NO cubre el barrido completo (t_settle %.1f + %zu vel x %.1f s "
          "= %.1f s): el barrido se TRUNCARA. Use duration_s:=%.1f (o duration_s:=auto en "
          "friction_sweep.launch.py).",
          duration_s_, t_settle_, vel_list_.size(), vel_seg_duration_, t_need,
          std::ceil(t_need + 2.0));
      }
    }

    motor_alpha_    = load_vec4("motor_alpha");
    motor_Fv_       = load_vec4("motor_Fv");
    motor_Fc_       = load_vec4("motor_Fc");
    motor_I_offset_ = load_vec4("motor_I_offset");
    motor_epsilon_  = get_parameter("motor_epsilon_friction").as_double();

    // ── Log de configuración ─────────────────────────────────────────────────
    RCLCPP_INFO(get_logger(),
      "hw=[port=%d torq=%d cmd=%d]  settle=%.1fs  ramp=%.1fs  dur=%.1fs  id=%d",
      (int)open_port_, (int)enable_torque_, (int)enable_current_commands_,
      t_settle_, t_ramp_, duration_s_, log_id);
    RCLCPP_INFO(get_logger(),
      "FASE SETTLING: todos los joints → pos_rad [%.3f, %.3f, %.3f, %.3f] rad (%.1f s)",
      pos_rad_(0), pos_rad_(1), pos_rad_(2), pos_rad_(3), t_settle_);
    for (int i = 0; i < NUM_JOINTS; ++i) {
      if (joint_mode_[i] == "current") {
        RCLCPP_INFO(get_logger(),
          "J%d [current]  A=%.3f  B=%.3f  w=%.3f rad/s  (inicio tras settling)",
          i+1, A_dc_(i), B_amp_(i), w_freq_(i));
        const double tau_pk = std::abs(A_dc_(i)) + std::abs(B_amp_(i));
        if (tau_pk > limits_.tau_max)
          RCLCPP_WARN(get_logger(),
            "J%d: |A|+|B|=%.3f N·m > TAU_MAX=%.3f N·m → se saturará", i+1, tau_pk, limits_.tau_max);
      } else if (joint_mode_[i] == "velocity") {
        std::string vl;
        for (double v : vel_list_) vl += std::to_string(v).substr(0, 4) + " ";
        RCLCPP_INFO(get_logger(),
          "J%d [velocity] barrido=[%s] rad/s  %.1fs c/u  banda=±%.2f rad alrededor de %.3f",
          i+1, vl.c_str(), vel_seg_duration_, vel_band_, A_pos_(i));
      } else {
        if (std::abs(B_pos_(i)) > 1e-9) {
          RCLCPP_INFO(get_logger(),
            "J%d [position] A=%.4f rad  B=%.4f rad  w=%.4f rad/s  (settling→%.4f rad)",
            i+1, A_pos_(i), B_pos_(i), w_pos_(i), pos_rad_(i));
        } else {
          RCLCPP_INFO(get_logger(),
            "J%d [position] setpoint=%.4f rad (%.2f°)  profile_vel=%d ticks",
            i+1, A_pos_(i), A_pos_(i)*180.0/PI, pos_profile_vel_);
        }
        const double abs_ticks = calib_.joint_zero_tick[i]
          + calib_.encoder_sign(i) * A_pos_(i) / POS_UNIT_RAD;
        if (abs_ticks < 0.0 || abs_ticks > 4095.0)
          RCLCPP_WARN(get_logger(),
            "J%d: A_pos=%.4f rad → %.1f ticks FUERA del rango [0,4095]",
            i+1, A_pos_(i), abs_ticks);
      }
    }

    // ── CSV de salida ────────────────────────────────────────────────────────
    std::filesystem::create_directories(
      std::string(PACKAGE_DATA_DIR) + "/diagnostics/sinusoidal");
    csv_path_ = std::string(PACKAGE_DATA_DIR)
      + "/diagnostics/sinusoidal/hw_sin_torque_" + std::to_string(log_id) + ".csv";
    csv_.open(csv_path_);
    if (csv_.is_open()) {
      csv_ << "t,q1,q2,q3,q4,dq1,dq2,dq3,dq4,"
              "dq_ref1,dq_ref2,dq_ref3,dq_ref4,"
              "ddq_ref1,ddq_ref2,ddq_ref3,ddq_ref4,"
              "tau_ref1,tau_ref2,tau_ref3,tau_ref4,"
              "pos_ref1,pos_ref2,pos_ref3,pos_ref4,"
              "curr_cmd1,curr_cmd2,curr_cmd3,curr_cmd4,"
              "curr_meas1,curr_meas2,curr_meas3,curr_meas4\n";
      RCLCPP_INFO(get_logger(), "CSV salida: %s", csv_path_.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "No se pudo crear CSV: %s", csv_path_.c_str());
    }

    // ── Publisher ────────────────────────────────────────────────────────────
    js_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/hw/joint_states", 10);

    // ── Hardware ─────────────────────────────────────────────────────────────
    if (open_port_) {
      if (!init_hardware()) {
        RCLCPP_FATAL(get_logger(), "Fallo hardware init.");
        throw std::runtime_error("Hardware init failed");
      }
    } else {
      RCLCPP_WARN(get_logger(), "MODO SECO (open_port=false) — sin hardware real.");
    }

    // ── Timer 100 Hz ─────────────────────────────────────────────────────────
    start_time_ = std::chrono::high_resolution_clock::now();
    ctrl_start_time_ = start_time_;  // se actualiza al terminar settling
    timer_       = this->create_wall_timer(10ms, [this]() { tick(); });
    RCLCPP_INFO(get_logger(),
      "Settling activo: %.1f s en posición inicial → luego control sinusoidal a 100 Hz.",
      t_settle_);
    if (duration_s_ > 0.0) {
      RCLCPP_INFO(get_logger(),
        "Al cumplir duration_s=%.1f s: retorno quintico a la posicion inicial "
        "(>= %.1f s) y pausa de %.1f s antes de cortar el torque.",
        duration_s_, RETURN_TIME_S, HOLD_TIME_S);
    } else {
      RCLCPP_WARN(get_logger(),
        "duration_s <= 0: corre hasta Ctrl+C SIN retorno (el torque se corta en la pose actual).");
    }
  }

  ~HWSinusoidalTorqueNode()
  {
    if (timer_) timer_->cancel();
    shutdown_hardware();
    if (csv_.is_open()) { csv_.flush(); csv_.close(); }
    RCLCPP_INFO(get_logger(), "Nodo finalizado.");
  }

private:
  // ─────────────────────────────────────────────────────────────────────────
  //  Generación de torque (current mode)
  //  t_ctrl: tiempo relativo al inicio del control (después del settling)
  // ─────────────────────────────────────────────────────────────────────────

  Vec4 compute_tau_ref(double t_ctrl) const
  {
    Vec4 tau = Vec4::Zero();
    for (int i = 0; i < NUM_JOINTS; ++i) {
      if (joint_mode_[i] != "current") continue;
      if (t_ctrl < t_ramp_) {
        tau(i) = (t_ctrl / t_ramp_) * A_dc_(i);
      } else {
        const double tp = t_ctrl - t_ramp_;
        tau(i) = A_dc_(i) + B_amp_(i) * std::sin(w_freq_(i) * tp);
      }
      tau(i) = std::clamp(tau(i), -limits_.tau_max, limits_.tau_max);
    }
    return tau;
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Generación de referencia de posición (position mode)
  //  q_ref_i(t_ctrl) = A_pos_i + B_pos_i * sin(w_pos_i * t_ctrl)
  // ─────────────────────────────────────────────────────────────────────────

  //  q_ref_i = A_pos_i + B_pos_i·sin(w_pos_i·t) + B_pos2_i·sin(w_pos2_i·t)
  //  El segundo armónico enriquece la distribución de velocidades (separa
  //  fricción viscosa de Coulomb en la identificación) y añade aceleración.
  Vec4 compute_pos_ref(double t_ctrl) const
  {
    Vec4 ref = Vec4::Zero();
    for (int i = 0; i < NUM_JOINTS; ++i) {
      if (joint_mode_[i] != "position") continue;
      ref(i) = A_pos_(i)
             + B_pos_(i)  * std::sin(w_pos_(i)  * t_ctrl)
             + B_pos2_(i) * std::sin(w_pos2_(i) * t_ctrl);
    }
    return ref;
  }

  // dq_ref_i = B·w·cos(w·t) + B2·w2·cos(w2·t)
  Vec4 compute_vel_ref(double t_ctrl) const
  {
    Vec4 ref = Vec4::Zero();
    for (int i = 0; i < NUM_JOINTS; ++i) {
      if (joint_mode_[i] != "position") continue;
      ref(i) = B_pos_(i)  * w_pos_(i)  * std::cos(w_pos_(i)  * t_ctrl)
             + B_pos2_(i) * w_pos2_(i) * std::cos(w_pos2_(i) * t_ctrl);
    }
    return ref;
  }

  // ddq_ref_i = -B·w²·sin(w·t) - B2·w2²·sin(w2·t)
  Vec4 compute_acc_ref(double t_ctrl) const
  {
    Vec4 ref = Vec4::Zero();
    for (int i = 0; i < NUM_JOINTS; ++i) {
      if (joint_mode_[i] != "position") continue;
      ref(i) = -B_pos_(i)  * w_pos_(i)  * w_pos_(i)  * std::sin(w_pos_(i)  * t_ctrl)
               -B_pos2_(i) * w_pos2_(i) * w_pos2_(i) * std::sin(w_pos2_(i) * t_ctrl);
    }
    return ref;
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Barrido de velocidad constante (mode="velocity")
  //  La magnitud avanza por vel_list_ cada vel_seg_duration_ segundos.
  //  El signo rebota dentro de [A_pos_i - vel_band_, A_pos_i + vel_band_] para
  //  permanecer cerca de una configuración conocida (gravedad acotada) sin chocar.
  //  Devuelve la velocidad objetivo [rad/s] por joint y actualiza vel_dir_.
  // ─────────────────────────────────────────────────────────────────────────

  Vec4 compute_vel_sweep(double t_ctrl, const Vec4& q)
  {
    Vec4 v = Vec4::Zero();
    const std::size_t idx = static_cast<std::size_t>(std::floor(t_ctrl / vel_seg_duration_));
    if (idx >= vel_list_.size()) return v;          // barrido terminado → 0
    const double mag = std::abs(vel_list_[idx]);
    for (int i = 0; i < NUM_JOINTS; ++i) {
      if (joint_mode_[i] != "velocity") continue;
      if (q(i) >= A_pos_(i) + vel_band_) vel_dir_[i] = -1.0;
      if (q(i) <= A_pos_(i) - vel_band_) vel_dir_[i] = +1.0;
      v(i) = vel_dir_[i] * mag;
    }
    return v;
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Conversión torque → corriente (modelo OLS identificado)
  // ─────────────────────────────────────────────────────────────────────────

  std::array<int16_t, NUM_JOINTS> torque_to_current(const Vec4& tau, const Vec4& dq) const
  {
    std::array<int16_t, NUM_JOINTS> cmd{};
    for (int i = 0; i < NUM_JOINTS; ++i) {
      if (joint_mode_[i] != "current") { cmd[i] = 0; continue; }
      const double I = motor_alpha_(i) * tau(i)
                     + motor_Fv_(i)    * dq(i)
                     + motor_Fc_(i)    * std::tanh(dq(i) / motor_epsilon_)
                     + motor_I_offset_(i);
      cmd[i] = clampCurrent(calib_.current_sign(i) * calib_.encoder_sign(i) * I, limits_.current_cmd_limit[i]);
    }
    return cmd;
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Hardware — inicialización
  //  Todos los joints arrancan en POSITION_CONTROL_MODE para la fase settling.
  // ─────────────────────────────────────────────────────────────────────────

  bool init_hardware()
  {
    port_handler_   = dynamixel::PortHandler::getPortHandler(port_name_.c_str());
    packet_handler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    if (!port_handler_->openPort()) {
      RCLCPP_ERROR(get_logger(), "No se pudo abrir %s", port_name_.c_str());
      return false;
    }
    if (!port_handler_->setBaudRate(BAUDRATE)) {
      RCLCPP_ERROR(get_logger(), "No se pudo configurar baudrate");
      port_handler_->closePort();
      return false;
    }

    // Deshabilitar todos antes de cambiar modo
    for (const auto id : DXL_ID)
      dxl_write1(id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE_VAL, "pre-disable");

    if (enable_torque_) {
      // TODOS los joints arrancan en Position Control Mode (para settling)
      for (int i = 0; i < NUM_JOINTS; ++i) {
        const uint8_t id = DXL_ID[i];
        if (!dxl_write1(id, ADDR_OPERATING_MODE, POSITION_CONTROL_MODE, "set pos mode") ||
            !dxl_write2(id, ADDR_CURRENT_LIMIT, limits_.current_limit_register, "set limit"))
          { port_handler_->closePort(); return false; }
        if (!dxl_write4(id, ADDR_PROFILE_ACC, static_cast<uint32_t>(pos_profile_acc_),
                        "set profile acc") ||
            !dxl_write4(id, ADDR_PROFILE_VEL, static_cast<uint32_t>(pos_profile_vel_),
                        "set profile vel"))
          { port_handler_->closePort(); return false; }
        if (!dxl_write1(id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE_VAL, "enable"))
          { port_handler_->closePort(); return false; }
        RCLCPP_INFO(get_logger(),
          "DXL ID %d → POSITION mode (settling)  target=%.4f rad",
          static_cast<int>(id), pos_rad_(i));
      }
    } else {
      RCLCPP_WARN(get_logger(), "enable_torque=false — motores en modo lectura.");
    }

    // SyncRead del bloque de estado (corriente+velocidad+posicion, todos los joints)
    reader_ = std::make_unique<StateReader>(port_handler_, packet_handler_, get_logger());

    // SyncWrite — grupos separados
    grp_wcur_ = std::make_unique<dynamixel::GroupSyncWrite>(
      port_handler_, packet_handler_, ADDR_GOAL_CURRENT,  LEN_GOAL_CURRENT);
    grp_wpos_ = std::make_unique<dynamixel::GroupSyncWrite>(
      port_handler_, packet_handler_, ADDR_GOAL_POSITION, LEN_GOAL_POSITION);
    grp_wvel_ = std::make_unique<dynamixel::GroupSyncWrite>(
      port_handler_, packet_handler_, ADDR_GOAL_VELOCITY, LEN_GOAL_VELOCITY);

    hw_active_ = true;
    RCLCPP_INFO(get_logger(), "Hardware inicializado en %s", port_name_.c_str());
    return true;
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Cambio de modo: joints "current" → CURRENT_CONTROL_MODE
  //  Se llama una sola vez al terminar la fase SETTLING.
  // ─────────────────────────────────────────────────────────────────────────

  void switch_to_run_modes()
  {
    for (int i = 0; i < NUM_JOINTS; ++i) {
      const uint8_t id = DXL_ID[i];
      if (joint_mode_[i] == "current") {
        dxl_write1(id, ADDR_TORQUE_ENABLE,  TORQUE_DISABLE_VAL,     "mode-switch disable");
        dxl_write1(id, ADDR_OPERATING_MODE, CURRENT_CONTROL_MODE,   "set current mode");
        dxl_write2(id, ADDR_CURRENT_LIMIT,  limits_.current_limit_register, "set limit");
        dxl_write1(id, ADDR_TORQUE_ENABLE,  TORQUE_ENABLE_VAL,      "mode-switch enable");
        RCLCPP_INFO(get_logger(), "DXL ID %d → CURRENT mode", static_cast<int>(id));
      } else if (joint_mode_[i] == "velocity") {
        dxl_write1(id, ADDR_TORQUE_ENABLE,  TORQUE_DISABLE_VAL,     "mode-switch disable");
        dxl_write1(id, ADDR_OPERATING_MODE, VELOCITY_CONTROL_MODE,  "set velocity mode");
        dxl_write2(id, ADDR_CURRENT_LIMIT,  limits_.current_limit_register, "set limit");
        dxl_write1(id, ADDR_TORQUE_ENABLE,  TORQUE_ENABLE_VAL,      "mode-switch enable");
        RCLCPP_INFO(get_logger(), "DXL ID %d → VELOCITY mode", static_cast<int>(id));
      }
    }
    run_modes_active_ = true;
    RCLCPP_INFO(get_logger(), "Settling completado → iniciando fase de control.");
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Retorno final: joint i → POSITION_CONTROL_MODE sosteniendo q_hold [rad]
  //  Goal Position (RAM) se escribe con el torque apagado y ANTES de
  //  re-habilitarlo, para que el servo no salte a una meta antigua; los
  //  perfiles se re-escriben después (cambiar de modo puede reiniciarlos).
  //  El joint queda sin torque unos pocos ms, igual que en el cambio de modo
  //  al terminar el settling.
  // ─────────────────────────────────────────────────────────────────────────

  void switch_to_position_hold(int i, double q_hold)
  {
    const uint8_t id = DXL_ID[i];
    dxl_write1(id, ADDR_TORQUE_ENABLE,  TORQUE_DISABLE_VAL,    "hold disable");
    dxl_write1(id, ADDR_OPERATING_MODE, POSITION_CONTROL_MODE, "set pos mode (retorno)");
    dxl_write4(id, ADDR_GOAL_POSITION,
               static_cast<uint32_t>(radToTicks(calib_, i, q_hold)), "goal hold");
    dxl_write1(id, ADDR_TORQUE_ENABLE,  TORQUE_ENABLE_VAL,     "hold enable");
    dxl_write4(id, ADDR_PROFILE_ACC, static_cast<uint32_t>(pos_profile_acc_), "profile acc");
    dxl_write4(id, ADDR_PROFILE_VEL, static_cast<uint32_t>(pos_profile_vel_), "profile vel");
    goal_pos_(i) = q_hold;
    RCLCPP_INFO(get_logger(), "DXL ID %d → POSITION mode (retorno), sosteniendo %.3f rad",
      static_cast<int>(id), q_hold);
  }

  // Fin del ensayo (t ≥ duration_s): los joints en current pasan a sostener
  // su pose; si hay joints en velocity se frenan primero (BRAKE), si no se
  // inicia directamente el retorno.
  void begin_end_sequence(double t, const Vec4& q)
  {
    RCLCPP_INFO(get_logger(),
      "Duración completada (%.1f s) → retorno a la posición inicial.", t);
    const bool cmd_ok = hw_active_ && enable_torque_ && enable_current_commands_;
    bool any_vel = false;
    if (run_modes_active_) {
      for (int i = 0; i < NUM_JOINTS; ++i) {
        if (joint_mode_[i] == "current" && cmd_ok) switch_to_position_hold(i, q(i));
        if (joint_mode_[i] == "velocity") any_vel = true;
      }
    }
    if (any_vel) {
      phase_    = Phase::BRAKE;
      t_phase0_ = t;
    } else {
      start_return(t);
    }
  }

  // Retorno quíntico goal_pos_ → q_initial_ (todos los joints en position mode).
  // Duración: RETURN_TIME_S como en los nodos hw_ de FL, alargada si hace falta
  // para que la velocidad pico de la quíntica (1.875·Δ/T) no supere RETURN_VMAX.
  void start_return(double t)
  {
    q_ret0_ = goal_pos_;
    const double dmax = (q_initial_ - q_ret0_).cwiseAbs().maxCoeff();
    T_ret_    = std::max(RETURN_TIME_S, 1.875 * dmax / RETURN_VMAX);
    t_phase0_ = t;
    phase_    = Phase::RETURN;
    RCLCPP_INFO(get_logger(),
      "Iniciando retorno quintico a q_inicial=[%.3f %.3f %.3f %.3f] rad "
      "(t=%.1fs, dur=%.1fs, |Δq|max=%.2f rad)",
      q_initial_(0), q_initial_(1), q_initial_(2), q_initial_(3), t, T_ret_, dmax);
  }

  void shutdown_hardware()
  {
    if (!hw_active_) return;
    hw_active_ = false;

    // Sin retorno (Ctrl+C o parada de emergencia durante el ensayo): anular
    // corriente/velocidad antes de cortar torque. En RETURN/HOLD todos los
    // joints ya están en position mode → torque off directo.
    if (phase_ == Phase::RAMP || phase_ == Phase::RUN) {
      const std::array<int16_t, NUM_JOINTS> zero{};
      if (grp_wcur_) send_currents(zero);
      if (grp_wvel_) send_velocities(Vec4::Zero());   // frenar joints en velocidad
      rclcpp::sleep_for(std::chrono::milliseconds(20));
    } else if (phase_ == Phase::BRAKE) {
      if (grp_wvel_) send_velocities(Vec4::Zero());
      rclcpp::sleep_for(std::chrono::milliseconds(20));
    }
    for (const auto id : DXL_ID)
      dxl_write1(id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE_VAL, "shutdown");
    if (port_handler_) {
      port_handler_->closePort();
      RCLCPP_INFO(get_logger(), "Puerto cerrado.");
    }
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Primitivas Dynamixel
  // ─────────────────────────────────────────────────────────────────────────

  bool dxl_write1(uint8_t id, uint16_t addr, uint8_t val, const char* lbl)
  {
    uint8_t err = 0;
    const int r = packet_handler_->write1ByteTxRx(port_handler_, id, addr, val, &err);
    if (r != COMM_SUCCESS || err != 0) {
      RCLCPP_WARN(get_logger(), "[ID %d] %s: r=%d err=%d", id, lbl, r, err);
      return false;
    }
    return true;
  }

  bool dxl_write2(uint8_t id, uint16_t addr, uint16_t val, const char* lbl)
  {
    uint8_t err = 0;
    const int r = packet_handler_->write2ByteTxRx(port_handler_, id, addr, val, &err);
    if (r != COMM_SUCCESS || err != 0) {
      RCLCPP_WARN(get_logger(), "[ID %d] %s: r=%d err=%d", id, lbl, r, err);
      return false;
    }
    return true;
  }

  bool dxl_write4(uint8_t id, uint16_t addr, uint32_t val, const char* lbl)
  {
    uint8_t err = 0;
    const int r = packet_handler_->write4ByteTxRx(port_handler_, id, addr, val, &err);
    if (r != COMM_SUCCESS || err != 0) {
      RCLCPP_WARN(get_logger(), "[ID %d] %s: r=%d err=%d", id, lbl, r, err);
      return false;
    }
    return true;
  }

  // Envía goal position a TODOS los joints (settling y retorno final)
  bool send_positions_all(const Vec4& target)
  {
    uint8_t p[NUM_JOINTS][4];
    for (int i = 0; i < NUM_JOINTS; ++i) {
      positionToBytes(radToTicks(calib_, i, target(i)), p[i]);
      grp_wpos_->addParam(DXL_ID[i], p[i]);
    }
    const int r = grp_wpos_->txPacket();
    grp_wpos_->clearParam();
    if (r != COMM_SUCCESS) {
      RCLCPP_ERROR(get_logger(), "SyncWrite (position all) fallo");
      return false;
    }
    goal_pos_ = target;
    return true;
  }

  // Envía goal position a joints en modo "position" (fase control)
  // pos_ref: referencia sinusoidal calculada por compute_pos_ref()
  bool send_positions(const Vec4& pos_ref)
  {
    uint8_t p[NUM_JOINTS][4];
    bool any = false;
    for (int i = 0; i < NUM_JOINTS; ++i) {
      if (joint_mode_[i] != "position") continue;
      positionToBytes(radToTicks(calib_, i, pos_ref(i)), p[i]);
      grp_wpos_->addParam(DXL_ID[i], p[i]);
      any = true;
    }
    if (!any) return true;
    const int r = grp_wpos_->txPacket();
    grp_wpos_->clearParam();
    if (r != COMM_SUCCESS) {
      RCLCPP_ERROR(get_logger(), "SyncWrite (position) fallo");
      return false;
    }
    for (int i = 0; i < NUM_JOINTS; ++i)
      if (joint_mode_[i] == "position") goal_pos_(i) = pos_ref(i);
    return true;
  }

  // Envía goal velocity solo a joints en modo "velocity"  (vel_cmd en rad/s)
  bool send_velocities(const Vec4& vel_cmd)
  {
    uint8_t p[NUM_JOINTS][4];
    bool any = false;
    for (int i = 0; i < NUM_JOINTS; ++i) {
      if (joint_mode_[i] != "velocity") continue;
      const int32_t ticks = static_cast<int32_t>(std::lround(
        calib_.encoder_sign(i) * vel_cmd(i) / VEL_UNIT_RAD_S));
      velocityToBytes(ticks, p[i]);
      grp_wvel_->addParam(DXL_ID[i], p[i]);
      any = true;
    }
    if (!any) return true;
    const int r = grp_wvel_->txPacket();
    grp_wvel_->clearParam();
    if (r != COMM_SUCCESS) {
      RCLCPP_ERROR(get_logger(), "SyncWrite (velocity) fallo");
      return false;
    }
    return true;
  }

  // Envía corriente solo a joints en modo "current"
  bool send_currents(const std::array<int16_t, NUM_JOINTS>& cmd)
  {
    uint8_t p[NUM_JOINTS][2];
    bool any = false;
    for (int i = 0; i < NUM_JOINTS; ++i) {
      if (joint_mode_[i] != "current") continue;
      currentToBytes(cmd[i], p[i]);
      grp_wcur_->addParam(DXL_ID[i], p[i]);
      any = true;
    }
    if (!any) return true;
    const int r = grp_wcur_->txPacket();
    grp_wcur_->clearParam();
    if (r != COMM_SUCCESS) {
      RCLCPP_ERROR(get_logger(), "SyncWrite (current) fallo");
      return false;
    }
    return true;
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Parada de emergencia
  // ─────────────────────────────────────────────────────────────────────────

  void emergency_stop(const std::string& reason)
  {
    RCLCPP_ERROR(get_logger(), "PARADA: %s", reason.c_str());
    timer_->cancel();
    shutdown_hardware();
    if (csv_.is_open()) { csv_.flush(); csv_.close(); }
    rclcpp::shutdown();
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Callback del timer (100 Hz)
  // ─────────────────────────────────────────────────────────────────────────

  void tick()
  {
    const auto   tp = std::chrono::high_resolution_clock::now();
    const double t  = std::chrono::duration<double>(tp - start_time_).count();

    // ── 1. Lectura de estado ──────────────────────────────────────────────
    Vec4 q = Vec4::Zero(), dq = Vec4::Zero();
    std::array<int16_t, NUM_JOINTS> cur_meas{};
    if (hw_active_) {
      if (!reader_->read(calib_, q, dq, cur_meas)) {
        emergency_stop("SyncRead fallido"); return;
      }
    }

    // Posición inicial = destino del retorno final. Se captura en la primera
    // lectura, ANTES de que el settling mueva el robot.
    if (!q_initial_captured_) {
      q_initial_ = q;
      q_initial_captured_ = true;
      RCLCPP_INFO(get_logger(), "q_inicial=[%.3f %.3f %.3f %.3f] rad (destino del retorno final)",
        q(0), q(1), q(2), q(3));
    }

    // Fin del ensayo → secuencia de retorno (BRAKE → RETURN → HOLD)
    const bool in_test = (phase_ == Phase::SETTLING || phase_ == Phase::RAMP ||
                          phase_ == Phase::RUN);
    if (in_test && duration_s_ > 0.0 && t >= duration_s_) begin_end_sequence(t, q);
    const bool cmd_ok   = hw_active_ && enable_torque_ && enable_current_commands_;
    const bool log_row  = (phase_ == Phase::SETTLING || phase_ == Phase::RAMP ||
                           phase_ == Phase::RUN);   // el retorno no se registra

    // ── 2. Lógica de fase ─────────────────────────────────────────────────
    Vec4 tau_ref = Vec4::Zero();
    std::array<int16_t, NUM_JOINTS> cur_cmd{};
    Vec4 pos_cmd_rad = Vec4::Zero();
    Vec4 dq_ref = Vec4::Zero(), ddq_ref = Vec4::Zero();

    if (phase_ == Phase::SETTLING) {
      // Enviar posición a TODOS los joints
      pos_cmd_rad = pos_rad_;
      if (hw_active_ && enable_current_commands_) {
        if (!send_positions_all(pos_rad_)) { emergency_stop("SyncWrite pos (settling) fallo"); return; }
      }

      // Transición al control cuando se cumple t_settle. Con enable_torque=false
      // (modo lectura) no se cambia de modo: eso re-habilitaría el torque.
      if (t >= t_settle_) {
        if (hw_active_ && enable_torque_) switch_to_run_modes();
        ctrl_start_time_ = tp;
        phase_ = Phase::RAMP;
      }

    } else if (phase_ == Phase::BRAKE) {
      // Frenar los joints en velocity; al cumplir BRAKE_TIME_S (ya detenidos)
      // pasan a position mode sosteniendo su pose y arranca el retorno.
      if (cmd_ok && !send_velocities(Vec4::Zero())) {
        emergency_stop("SyncWrite velocity (frenado) fallo"); return;
      }
      if (t >= t_phase0_ + BRAKE_TIME_S) {
        if (cmd_ok) {
          for (int i = 0; i < NUM_JOINTS; ++i)
            if (joint_mode_[i] == "velocity") switch_to_position_hold(i, q(i));
        }
        start_return(t);
      }

    } else if (phase_ == Phase::RETURN) {
      // Quíntica 10s³-15s⁴+6s⁵ (vel. y acel. nulas en ambos extremos)
      const double s = std::clamp((t - t_phase0_) / T_ret_, 0.0, 1.0);
      const double h = quinticProfile(s);
      pos_cmd_rad = q_ret0_ + h * (q_initial_ - q_ret0_);
      if (cmd_ok && !send_positions_all(pos_cmd_rad)) {
        emergency_stop("SyncWrite position (retorno) fallo"); return;
      }
      if (s >= 1.0) { phase_ = Phase::HOLD; t_phase0_ = t; }

    } else if (phase_ == Phase::HOLD) {
      pos_cmd_rad = q_initial_;
      if (cmd_ok && !send_positions_all(q_initial_)) {
        emergency_stop("SyncWrite position (pausa final) fallo"); return;
      }
      if (t >= t_phase0_ + HOLD_TIME_S) {
        const double err = (q - q_initial_).cwiseAbs().maxCoeff();
        RCLCPP_INFO(get_logger(),
          "Retorno a q_inicial completado (|q - q_inicial|max = %.3f rad). "
          "Cortando torque (t=%.1f s).", err, t);
        if (hw_active_ && err > 0.10)
          RCLCPP_WARN(get_logger(),
            "El robot quedó a %.2f rad de q_inicial (¿obstáculo o tope mecánico?).", err);
        timer_->cancel();
        shutdown_hardware();
        if (csv_.is_open()) { csv_.flush(); csv_.close(); }
        rclcpp::shutdown();
        return;
      }

    } else {
      // Fases RAMP y RUN
      const double t_ctrl = std::chrono::duration<double>(tp - ctrl_start_time_).count();
      phase_ = (t_ctrl < t_ramp_) ? Phase::RAMP : Phase::RUN;

      tau_ref     = compute_tau_ref(t_ctrl);
      pos_cmd_rad = compute_pos_ref(t_ctrl);
      dq_ref      = compute_vel_ref(t_ctrl);
      ddq_ref     = compute_acc_ref(t_ctrl);
      cur_cmd     = torque_to_current(tau_ref, dq);

      // Barrido de velocidad constante (joints en mode="velocity").
      // Se registra la velocidad comandada en dq_ref y ddq_ref=0 (constante).
      const Vec4 vel_cmd = compute_vel_sweep(t_ctrl, q);
      for (int i = 0; i < NUM_JOINTS; ++i) {
        if (joint_mode_[i] == "velocity") { dq_ref(i) = vel_cmd(i); ddq_ref(i) = 0.0; }
      }

      if (hw_active_ && enable_current_commands_) {
        if (!send_currents(cur_cmd))        { emergency_stop("SyncWrite current fallo"); return; }
        if (!send_positions(pos_cmd_rad))   { emergency_stop("SyncWrite position fallo"); return; }
        if (!send_velocities(vel_cmd))      { emergency_stop("SyncWrite velocity fallo"); return; }
      }
    }

    // ── 3. Verificación de corriente medida ───────────────────────────────
    if (hw_active_) {
      for (int i = 0; i < NUM_JOINTS; ++i) {
        if (std::abs(cur_meas[i]) > limits_.current_measured_peak) {
          emergency_stop("Corriente insegura J" + std::to_string(i + 1)
                         + ": " + std::to_string(cur_meas[i]) + " ticks");
          return;
        }
      }
    }

    // ── 4. Publicar JointState ────────────────────────────────────────────
    {
      sensor_msgs::msg::JointState js;
      js.header.stamp = this->now();
      js.name         = {"joint1","joint2","joint3","joint4"};
      js.position     = {q(0),  q(1),  q(2),  q(3)};
      js.velocity     = {dq(0), dq(1), dq(2), dq(3)};
      js.effort       = {tau_ref(0), tau_ref(1), tau_ref(2), tau_ref(3)};
      js_pub_->publish(js);
    }

    // ── 5. CSV (solo SETTLING/RAMP/RUN: el retorno final no se registra) ──
    if (csv_.is_open() && log_row) {
      csv_ << std::fixed << std::setprecision(6) << t
           << ',' << q(0)           << ',' << q(1)           << ',' << q(2)           << ',' << q(3)
           << ',' << dq(0)          << ',' << dq(1)          << ',' << dq(2)          << ',' << dq(3)
           << ',' << dq_ref(0)      << ',' << dq_ref(1)      << ',' << dq_ref(2)      << ',' << dq_ref(3)
           << ',' << ddq_ref(0)     << ',' << ddq_ref(1)     << ',' << ddq_ref(2)     << ',' << ddq_ref(3)
           << ',' << tau_ref(0)     << ',' << tau_ref(1)     << ',' << tau_ref(2)     << ',' << tau_ref(3)
           << ',' << pos_cmd_rad(0) << ',' << pos_cmd_rad(1) << ',' << pos_cmd_rad(2) << ',' << pos_cmd_rad(3)  // pos_ref
           << ',' << cur_cmd[0]     << ',' << cur_cmd[1]     << ',' << cur_cmd[2]     << ',' << cur_cmd[3]
           << ',' << cur_meas[0]    << ',' << cur_meas[1]    << ',' << cur_meas[2]    << ',' << cur_meas[3]
           << '\n';
    }

    // ── 6. Log periódico ──────────────────────────────────────────────────
    if (++log_cnt_ % 100 == 0) {
      if (csv_.is_open()) csv_.flush();
      const char* fase = (phase_ == Phase::SETTLING) ? "SETTL"
                       : (phase_ == Phase::RAMP)     ? "RAMP "
                       : (phase_ == Phase::RUN)      ? "RUN  "
                       : (phase_ == Phase::BRAKE)    ? "BRAKE"
                       : (phase_ == Phase::RETURN)   ? "RETUR"
                                                     : "HOLD ";
      RCLCPP_INFO(get_logger(),
        "[%s] t=%.2fs  q=[%.3f %.3f %.3f %.3f] rad  τ=[%.3f %.3f %.3f %.3f] N·m",
        fase, t, q(0), q(1), q(2), q(3),
        tau_ref(0), tau_ref(1), tau_ref(2), tau_ref(3));
    }
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Miembros
  // ─────────────────────────────────────────────────────────────────────────

  std::string port_name_;
  bool open_port_, enable_torque_, enable_current_commands_;
  double t_settle_, t_ramp_, duration_s_;

  std::vector<std::string> joint_mode_;
  Vec4 A_dc_, B_amp_, w_freq_;             // parámetros señal de torque (current mode)
  Vec4 A_pos_, B_pos_, w_pos_;             // parámetros trayectoria de posición (position mode)
  Vec4 B_pos2_, w_pos2_;                    // segundo armónico (excitación multi-velocidad)
  Vec4 pos_rad_;                           // posición inicial de settling (todos los joints)
  int  pos_profile_vel_, pos_profile_acc_;

  std::vector<double> vel_list_;                       // magnitudes de velocidad a barrer
  double vel_seg_duration_, vel_band_;                 // duración por magnitud / semirango
  std::array<double, NUM_JOINTS> vel_dir_{};           // signo actual del rebote por joint

  Vec4   motor_alpha_, motor_Fv_, motor_Fc_, motor_I_offset_;
  double motor_epsilon_;
  Calibration calib_;    // ceros y signos (YAML del motor)
  HwLimits    limits_;   // limites de corriente y torque (YAML del motor)

  dynamixel::PortHandler*   port_handler_{nullptr};
  dynamixel::PacketHandler* packet_handler_{nullptr};
  std::unique_ptr<StateReader>               reader_;
  std::unique_ptr<dynamixel::GroupSyncWrite> grp_wcur_, grp_wpos_, grp_wvel_;

  bool  hw_active_;
  Phase phase_;
  bool  run_modes_active_{false};          // joints current/velocity ya cambiados de modo

  // Retorno final a la posición inicial
  Vec4   q_initial_ = Vec4::Zero();        // pose leída en el primer ciclo (destino)
  bool   q_initial_captured_{false};
  Vec4   goal_pos_  = Vec4::Zero();        // meta de posición vigente por joint
  Vec4   q_ret0_    = Vec4::Zero();        // inicio de la quíntica de retorno
  double T_ret_{RETURN_TIME_S};            // duración efectiva del retorno [s]
  double t_phase0_{0.0};                   // inicio de la fase BRAKE/RETURN/HOLD [s]

  std::string   csv_path_;
  std::ofstream csv_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::chrono::high_resolution_clock::time_point start_time_;
  std::chrono::high_resolution_clock::time_point ctrl_start_time_;
  int log_cnt_{0};
};

// ============================================================
// main — carga automática del YAML de configuración
// ============================================================

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions opts;
  const std::string cfg = std::string(PACKAGE_CONFIG_DIR) + "/hw_sinusoidal_torque_params.yaml";

  if (std::filesystem::exists(cfg)) {
    // Construimos la lista de args combinando: YAML primero, luego los --ros-args
    // del usuario. Así los -p pasados por línea de comandos sobreescriben el YAML.
    std::vector<std::string> args = {"--ros-args", "--params-file", cfg};
    bool in_ros_args = false;
    for (int i = 1; i < argc; ++i) {
      const std::string a(argv[i]);
      if (a == "--ros-args") { in_ros_args = true; continue; }
      if (in_ros_args) args.push_back(a);
    }
    opts.arguments(args);
    opts.use_global_arguments(false);   // evitar doble procesamiento
    RCLCPP_INFO(rclcpp::get_logger("main"), "Config auto-cargado: %s", cfg.c_str());
  } else {
    RCLCPP_WARN(rclcpp::get_logger("main"),
      "Config no encontrado: %s — usando defaults del código.", cfg.c_str());
  }

  try {
    rclcpp::spin(std::make_shared<HWSinusoidalTorqueNode>(opts));
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("main"), "Excepcion: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
