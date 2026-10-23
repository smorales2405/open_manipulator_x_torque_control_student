/*
 * hw_alpha_ident_node.cpp
 * Ensayo de identificación de alpha e I_offset (Paso 2 del protocolo
 * src/Identification/Ident_OpenManX_XM430W350T_procedure.md) — OpenMANIPULATOR-X
 * hardware real vía Dynamixel SDK directo, sin ros2_control.
 *
 * Qué se mide: la cadena  tau → corriente COMANDADA → corriente REAL.
 * El nodo comanda  I_cmd = ALPHA_NOM·tau  (conversión nominal del datasheet,
 * SIN fricción ni offset) y registra la corriente medida; identify_alpha.py
 * ajusta el lazo interno de corriente del Dynamixel, I_meas ≈ k·I_cmd + b, y
 * entrega  alpha = ALPHA_NOM/k,  I_offset = −b/k.
 *
 * Por diseño NO lee motor_alpha/Fv/Fc/I_offset de motorXM430W350T_params.yaml
 * (solo el bloque de hardware: ceros, signos, límites): el ensayo usa siempre
 * la conversión nominal, sin editar el YAML, y no depende del nodo de Feedback
 * Linearization (los alumnos identifican antes de implementar FL).
 *
 * Ley de control — PD articular + compensación de gravedad (G del URDF):
 *   tau = G(q) + Kp·(q_des − q) + Kd·(dq_des − dq_hat)
 * El seguimiento no necesita ser bueno (sin compensación de fricción hay
 * stick-slip visible): el análisis usa solo corriente comandada vs medida.
 * Lo que importa es que la corriente comandada varíe: J2 por gravedad, J3 por
 * la fricción que el PD vence en cada inversión, J4 por la fricción y la
 * gravedad del efector (de ahí su oscilación).
 *
 * Trayectoria: la act1 del laboratorio de FL (senos de 1 rad/s en J1–J3,
 * probada en el robot real dentro de límites y sin colisiones) con J4
 * oscilando ±0.25 rad alrededor de π/4 EN FASE con J1–J3: en act1 J4 queda
 * fijo y su corriente casi constante (k no identificable). En fase, la pinza
 * queda más lejos del robot que con J4 fijo (URDF: altura mínima 121 → 136 mm).
 *
 * Fases temporales (igual que los nodos hw_ de FL):
 *   [0, RAMP_TIME_S)                        rampa quíntica: reposo → trayectoria
 *   [RAMP_TIME_S, t_run)                    trayectoria articular periódica
 *   [t_run, t_run+RETURN_TIME_S)            retorno quíntico a q_inicial
 *   [t_run+RETURN_TIME_S, ...+HOLD_TIME_S)  pausa en reposo, luego torque off
 * El CSV solo registra la rampa y la trayectoria. Ctrl+C o una parada de
 * emergencia cortan el torque en el acto, sin retorno.
 *
 * Parámetros ROS 2 (--ros-args -p nombre:=valor):
 *   port_name     [string]    "/dev/ttyUSB0"
 *   open_port     [bool]      true   false = dry-run: recorre la trayectoria sin
 *                                    hardware, verifica límites y corrientes
 *                                    previstas, y termina (no escribe CSV)
 *   log_id        [int]       30     CSV: data/identification/alpha/hw_alpha_ident_<log_id>.csv
 *   t_run         [double]    40.0   [s] rampa + trayectoria (sin el retorno)
 *   gain_scale    [double]    1.0    escala lineal de Kp; Kd escala con sqrt
 *   kp            [double[4]] [2.5, 5.0, 5.0, 3.0]     [N·m/rad]
 *   kd            [double[4]] [0.17, 0.33, 0.28, 0.08] [N·m·s/rad]
 *   ab_alpha, ab_beta [double] 0.2, 0.02   filtro α-β de velocidad
 *   loop_rate_hz  [double]    200.0  se acota a [50, 400]
 *
 * Ejemplo (ver launch/alpha_ident.launch.py):
 *   ros2 launch open_manipulator_x_torque_control alpha_ident.launch.py log_id:=30
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

#include "omx/dynamixel_arm.hpp"
#include "omx/motor_model.hpp"   // HwLimits
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

// Conversión NOMINAL fija del ensayo: kt del datasheet XM430-W350 (stall
// 4.1 N·m / 2.3 A a 12 V) → ALPHA_NOM ≈ 208.5 ticks/N·m. Sin Fv/Fc/I_offset.
static constexpr double KT_DATASHEET_NM_A = 1.7826;
static constexpr double ALPHA_NOM = 1.0 / (KT_DATASHEET_NM_A * omx::CURRENT_UNIT_A);

// Calibración y límites: omx::declareCalibration / omx::declareHwLimits
// (bloque de hardware de config/motorXM430W350T_params.yaml).

static constexpr double RAMP_TIME_S   = 3.0;
static constexpr double RETURN_TIME_S = 3.0;   // duración del retorno quíntico a q_inicial
static constexpr double HOLD_TIME_S   = 0.5;   // pausa en reposo antes de cortar corriente
static constexpr double MAX_TRACK_ERR = 0.5;   // [rad] error de seguimiento → parada
                                               // (signos invertidos, colisión, atasco)

using Vec4 = Eigen::Matrix<double, NUM_JOINTS, 1>;

// ============================================================
// Trayectoria deseada (act1 de hw_fl_control_node con J4 oscilando)
// ============================================================

using omx::Reference;

static Reference desiredTrajectory(double t)
{
  const double w = 1.0;
  Reference r;
  r.q   << (PI/4.0)*std::sin(w*t),      -0.5 + 0.5*std::sin(w*t),    0.3 - 0.5*std::sin(w*t),   (PI/4.0) + 0.25*std::sin(w*t);
  r.dq  << (PI/4.0)*w*std::cos(w*t),     0.5*w*std::cos(w*t),        -0.5*w*std::cos(w*t),        0.25*w*std::cos(w*t);
  r.ddq << -(PI/4.0)*w*w*std::sin(w*t), -0.5*w*w*std::sin(w*t),      0.5*w*w*std::sin(w*t),      -0.25*w*w*std::sin(w*t);
  return r;
}

// Referencia completa: rampa desde el reposo en q0 → trayectoria periódica →
// retorno en reposo a q0 → pausa. t_run <= 0 desactiva el retorno.
static Reference referenceAt(double t, double t_run, const Vec4& q0)
{
  if (t < RAMP_TIME_S) {
    return omx::quinticTransition(t, RAMP_TIME_S, q0, desiredTrajectory(RAMP_TIME_S));
  }
  if (t_run <= 0.0 || t < t_run) return desiredTrajectory(t);
  if (t < t_run + RETURN_TIME_S) {
    return omx::quinticReturn(t - t_run, RETURN_TIME_S, desiredTrajectory(t_run), q0);
  }
  Reference r;
  r.q = q0; r.dq.setZero(); r.ddq.setZero();
  return r;
}

// ============================================================
// Nodo ROS 2
// ============================================================

class HWAlphaIdentNode : public rclcpp::Node
{
public:
  explicit HWAlphaIdentNode(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions())
  : Node("hw_alpha_ident_node", opts),
    q_initial_captured_(false)
  {
    // ── Parámetros ──────────────────────────────────────────────────────────
    using dvec = std::vector<double>;
    this->declare_parameter<std::string>("port_name",  "/dev/ttyUSB0");
    this->declare_parameter<bool>       ("open_port",  true);
    this->declare_parameter<int>        ("log_id",     30);
    this->declare_parameter<double>     ("t_run",      40.0);
    this->declare_parameter<double>     ("gain_scale", 1.0);
    // Con la inercia media del URDF sobre act1: ωn ≈ 20–50 rad/s y
    // ζ ≈ 0.5–1.0 en todo el recorrido.
    this->declare_parameter<dvec>("kp", dvec{2.5,  5.0,  5.0,  3.0 });
    this->declare_parameter<dvec>("kd", dvec{0.17, 0.33, 0.28, 0.08});
    this->declare_parameter<double>("ab_alpha", 0.2);
    this->declare_parameter<double>("ab_beta",  0.02);
    this->declare_parameter<double>("loop_rate_hz", 200.0);

    // Solo el bloque de hardware del YAML del motor (ceros, signos, límites):
    // el modelo del motor (motor_alpha/Fv/Fc/I_offset) NO se usa en este ensayo.
    calib_  = omx::declareCalibration(*this);
    limits_ = omx::declareHwLimits(*this);

    port_name_  = get_parameter("port_name").as_string();
    open_port_  = get_parameter("open_port").as_bool();
    log_id_     = static_cast<int>(get_parameter("log_id").as_int());
    t_run_      = get_parameter("t_run").as_double();
    gain_scale_ = get_parameter("gain_scale").as_double();

    auto load_vec4 = [this](const std::string& name) {
      auto v = get_parameter(name).as_double_array();
      return Vec4(v[0], v[1], v[2], v[3]);
    };
    kp_ = load_vec4("kp") * gain_scale_;
    kd_ = load_vec4("kd") * std::sqrt(std::max(gain_scale_, 0.0));

    ab_alpha_ = get_parameter("ab_alpha").as_double();
    ab_beta_  = get_parameter("ab_beta").as_double();
    loop_rate_hz_ = std::min(std::max(get_parameter("loop_rate_hz").as_double(), 50.0), 400.0);
    Ts_ = 1.0 / loop_rate_hz_;

    RCLCPP_INFO(get_logger(), "puerto=%s  open_port=%d  t_run=%.1fs  gain=%.2f  id=%d",
      port_name_.c_str(), open_port_ ? 1 : 0, t_run_, gain_scale_, log_id_);
    RCLCPP_INFO(get_logger(),
      "Conversión FIJA del ensayo: α=%.2f ticks/N·m (nominal), sin Fv/Fc/I_offset "
      "(el YAML de motor no se usa)", ALPHA_NOM);
    RCLCPP_INFO(get_logger(), "PD+G: Kp=[%.2f %.2f %.2f %.2f] N·m/rad  Kd=[%.3f %.3f %.3f %.3f] N·m·s/rad",
      kp_(0), kp_(1), kp_(2), kp_(3), kd_(0), kd_(1), kd_(2), kd_(3));
    RCLCPP_INFO(get_logger(),
      "hw: zero=[%d %d %d %d]  enc_sign=[%.0f %.0f %.0f %.0f]  cur_sign=[%.0f %.0f %.0f %.0f]",
      calib_.joint_zero_tick[0], calib_.joint_zero_tick[1], calib_.joint_zero_tick[2], calib_.joint_zero_tick[3],
      calib_.encoder_sign(0), calib_.encoder_sign(1), calib_.encoder_sign(2), calib_.encoder_sign(3),
      calib_.current_sign(0), calib_.current_sign(1), calib_.current_sign(2), calib_.current_sign(3));

    if (t_run_ <= RAMP_TIME_S + 10.0) {
      RCLCPP_WARN(get_logger(), "t_run=%.1f s es corto: identify_alpha.py descarta los "
        "primeros 4 s; use t_run ≥ 30 s.", t_run_);
    }

    // ── Pinocchio (solo G(q)) ────────────────────────────────────────────────
    const std::string urdf = std::string(PACKAGE_URDF_DIR) + "/open_manipulator_x.urdf";
    try {
      model_.load(urdf);
    } catch (const std::exception& e) {
      RCLCPP_FATAL(get_logger(), "Pinocchio URDF: %s", e.what());
      throw;
    }

    // Dry-run: sin hardware, sin CSV, sin timer (main llama a preflight()).
    if (!open_port_) return;

    // ── CSV ──────────────────────────────────────────────────────────────────
    const std::string dir = std::string(PACKAGE_DATA_DIR) + "/identification/alpha";
    csv_path_ = dir + "/hw_alpha_ident_" + std::to_string(log_id_) + ".csv";
    omx::openCsv(csv_, csv_path_,
      "t,q1,q2,q3,q4,dq1,dq2,dq3,dq4,dq1_filt,dq2_filt,dq3_filt,dq4_filt,"
      "q1_des,q2_des,q3_des,q4_des,dq1_des,dq2_des,dq3_des,dq4_des,"
      "tau1,tau2,tau3,tau4,tau_g1,tau_g2,tau_g3,tau_g4,"
      "curr_cmd1,curr_cmd2,curr_cmd3,curr_cmd4,"
      "curr_meas1,curr_meas2,curr_meas3,curr_meas4",
      get_logger());

    js_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/hw/joint_states", 10);

    // ── SDK ──────────────────────────────────────────────────────────────────
    if (!arm_.open(port_name_, calib_, limits_.current_limit_register)) {
      RCLCPP_FATAL(get_logger(), "Fallo hardware init. Abortando.");
      throw std::runtime_error("Hardware init failed");
    }

    start_time_ = std::chrono::high_resolution_clock::now();
    const auto period = std::chrono::microseconds(
      static_cast<int64_t>(std::lround(1e6 / loop_rate_hz_)));
    timer_ = this->create_wall_timer(period, [this]() { tick(); });
    RCLCPP_INFO(get_logger(), "Control activo a %.0f Hz. Duración ≈ %.1f s + retorno %.1f s. "
      "Ctrl+C corta el torque SIN retorno.", loop_rate_hz_, t_run_, RETURN_TIME_S + HOLD_TIME_S);
  }

  ~HWAlphaIdentNode()
  {
    if (timer_) timer_->cancel();
    arm_.close();
    if (csv_.is_open()) { csv_.flush(); csv_.close(); }
  }

  bool dry_run() const { return !open_port_; }

  // Dry-run: recorre la referencia completa con seguimiento perfecto
  // (q = q_des → tau = G(q_des)) partiendo del inicio de la trayectoria, y
  // reporta límites y corrientes previstas. En el robot real la rampa parte
  // de la pose en que esté el robot al lanzar el nodo.
  void preflight()
  {
    const Vec4 q0 = desiredTrajectory(0.0).q;
    const double t_end = t_run_ + RETURN_TIME_S + HOLD_TIME_S;
    Vec4 tau_pk = Vec4::Zero(), q_min = q0, q_max = q0;
    bool limits_ok = true;
    for (double t = 0.0; t < t_end; t += Ts_) {
      const Reference ref = referenceAt(t, t_run_, q0);
      q_min = q_min.cwiseMin(ref.q);
      q_max = q_max.cwiseMax(ref.q);
      for (int i = 0; i < NUM_JOINTS; ++i)
        if (ref.q(i) < limits_.joint_lower(i) + 0.02 || ref.q(i) > limits_.joint_upper(i) - 0.02) limits_ok = false;
      tau_pk = tau_pk.cwiseMax(gravity(ref.q).cwiseAbs());
    }
    RCLCPP_INFO(get_logger(), "══ DRY-RUN (open_port=false): no se abre el puerto ni se escribe CSV ══");
    RCLCPP_INFO(get_logger(), "Duración: %.1f s de ensayo (%d filas de CSV a %.0f Hz) + %.1f s de retorno",
      t_run_, static_cast<int>(t_run_ * loop_rate_hz_), loop_rate_hz_, RETURN_TIME_S + HOLD_TIME_S);
    for (int i = 0; i < NUM_JOINTS; ++i) {
      const double I_pk = ALPHA_NOM * tau_pk(i);
      RCLCPP_INFO(get_logger(), "J%d: q ∈ [%+.3f, %+.3f] rad (límites [%+.3f, %+.3f])  "
        "|G|max=%.3f N·m → |I_cmd| ≈ %.0f ticks (límite %d)%s",
        i + 1, q_min(i), q_max(i), limits_.joint_lower(i), limits_.joint_upper(i), tau_pk(i), I_pk,
        static_cast<int>(limits_.current_cmd_limit[i]),
        I_pk > 0.8 * limits_.current_cmd_limit[i] ? "  ← margen de corriente bajo" : "");
    }
    if (limits_ok) {
      RCLCPP_INFO(get_logger(), "Referencia dentro de límites articulares. Para el ensayo real "
        "quite open_port:=false (el robot SE MOVERÁ).");
    } else {
      RCLCPP_ERROR(get_logger(), "La referencia SALE de los límites articulares: revise "
        "joint_lower/joint_upper en motorXM430W350T_params.yaml.");
    }
  }

private:
  // ─────────────────────────────────────────────────────────────────────────
  //  Ley de control: PD articular + G(q)
  // ─────────────────────────────────────────────────────────────────────────

  Vec4 gravity(const Vec4& q)
  {
    return model_.gravity(q);
  }

  // Conversión fija del ensayo: I = ALPHA_NOM·tau, sin fricción ni offset.
  omx::Currents torque_to_current(const Vec4& tau)
  {
    omx::Currents cmd{};
    for (int i = 0; i < NUM_JOINTS; ++i)
      cmd[i] = omx::clampCurrent(calib_.current_sign(i) * calib_.encoder_sign(i) * ALPHA_NOM * tau(i),
                                 limits_.current_cmd_limit[i]);
    return cmd;
  }

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
    const double t = std::chrono::duration<double>(
      std::chrono::high_resolution_clock::now() - start_time_).count();

    // 1. Lectura de estado
    Vec4 q, dq;
    std::array<int16_t, NUM_JOINTS> cur_meas{};
    if (!arm_.read(q, dq, cur_meas)) {
      emergency_stop("SyncRead fallido");
      return;
    }

    if (t_run_ > 0.0 && t >= t_run_ + RETURN_TIME_S + HOLD_TIME_S) {
      RCLCPP_INFO(get_logger(), "Retorno completado: el robot quedó a %.3f rad de q_inicial. "
        "Torque off (t=%.1f s).", (q - q_initial_).norm(), t);
      RCLCPP_INFO(get_logger(), "Analizar: python3 src/Identification/identify_alpha.py %d",
        log_id_);
      timer_->cancel();
      arm_.close();
      if (csv_.is_open()) { csv_.flush(); csv_.close(); }
      rclcpp::shutdown();
      return;
    }

    // 1b. Filtro α-β de velocidad (como hw_fl_control_node)
    if (!ab_initialized_) {
      q_hat_  = q;
      dq_hat_ = dq;
      ab_initialized_ = true;
    } else {
      const Vec4 q_pred = q_hat_ + Ts_ * dq_hat_;
      const Vec4 r      = q - q_pred;
      q_hat_  = q_pred  + ab_alpha_ * r;
      dq_hat_ = dq_hat_ + (ab_beta_ / Ts_) * r;
    }

    // 2. Posición inicial (primer tick): origen de la rampa y destino del retorno
    if (!q_initial_captured_) {
      q_initial_ = q;
      q_initial_captured_ = true;
      RCLCPP_INFO(get_logger(), "q_inicial=[%.3f %.3f %.3f %.3f] rad",
        q(0), q(1), q(2), q(3));
    }

    // 3. Referencia
    if (t_run_ > 0.0 && t >= t_run_ && !return_logged_) {
      RCLCPP_INFO(get_logger(), "Fin del ensayo (t=%.1f s): retorno quíntico a q_inicial (%.1f s)",
        t, RETURN_TIME_S);
      return_logged_ = true;
    }
    const Reference ref = referenceAt(t, t_run_, q_initial_);

    // 4. Seguridad: referencia dentro de límites y error de seguimiento acotado
    for (int i = 0; i < NUM_JOINTS; ++i) {
      if (ref.q(i) < limits_.joint_lower(i) + 0.02 || ref.q(i) > limits_.joint_upper(i) - 0.02) {
        emergency_stop("Referencia fuera de límites articulares en J" + std::to_string(i + 1));
        return;
      }
      if (std::abs(q(i) - ref.q(i)) > MAX_TRACK_ERR) {
        emergency_stop("Error de seguimiento > " + std::to_string(MAX_TRACK_ERR)
                       + " rad en J" + std::to_string(i + 1)
                       + " (¿current_sign/encoder_sign invertidos o colisión?)");
        return;
      }
    }

    // 5. Ley de control PD + G y conversión nominal
    const Vec4 tau_g = gravity(q);
    const Vec4 tau_unsat = tau_g + kp_.cwiseProduct(ref.q - q) + kd_.cwiseProduct(ref.dq - dq_hat_);
    const Vec4 tau = tau_unsat.cwiseMin(limits_.tau_max).cwiseMax(-limits_.tau_max);
    const auto cur_cmd = torque_to_current(tau);

    // 6. Escribir corriente
    if (!arm_.writeCurrents(cur_cmd)) {
      emergency_stop("SyncWrite fallido");
      return;
    }

    // 7. Verificar corriente medida (seguridad)
    for (int i = 0; i < NUM_JOINTS; ++i) {
      if (std::abs(cur_meas[i]) > limits_.current_measured_peak) {
        emergency_stop("Corriente medida insegura en J" + std::to_string(i + 1)
                       + ": " + std::to_string(cur_meas[i]) + " ticks");
        return;
      }
    }

    // 8. JointState de monitoreo (effort = torque nominal de la corriente medida)
    {
      sensor_msgs::msg::JointState js;
      js.header.stamp = this->now();
      js.name         = {"joint1", "joint2", "joint3", "joint4"};
      js.position     = {q(0), q(1), q(2), q(3)};
      js.velocity     = {dq(0), dq(1), dq(2), dq(3)};
      js.effort       = {cur_meas[0] / ALPHA_NOM, cur_meas[1] / ALPHA_NOM,
                         cur_meas[2] / ALPHA_NOM, cur_meas[3] / ALPHA_NOM};
      js_pub_->publish(js);
    }

    // 9. CSV (solo rampa + trayectoria; el retorno y la pausa no se guardan).
    //    curr_meas de esta fila se leyó ANTES de escribir curr_cmd: responde
    //    al comando de la fila anterior (identify_alpha.py lo alinea).
    if (csv_.is_open() && !(t_run_ > 0.0 && t >= t_run_)) {
      csv_ << std::fixed << std::setprecision(6) << t
           << ',' << q(0)       << ',' << q(1)       << ',' << q(2)       << ',' << q(3)
           << ',' << dq(0)      << ',' << dq(1)      << ',' << dq(2)      << ',' << dq(3)
           << ',' << dq_hat_(0) << ',' << dq_hat_(1) << ',' << dq_hat_(2) << ',' << dq_hat_(3)
           << ',' << ref.q(0)   << ',' << ref.q(1)   << ',' << ref.q(2)   << ',' << ref.q(3)
           << ',' << ref.dq(0)  << ',' << ref.dq(1)  << ',' << ref.dq(2)  << ',' << ref.dq(3)
           << ',' << tau(0)     << ',' << tau(1)     << ',' << tau(2)     << ',' << tau(3)
           << ',' << tau_g(0)   << ',' << tau_g(1)   << ',' << tau_g(2)   << ',' << tau_g(3)
           << ',' << cur_cmd[0] << ',' << cur_cmd[1] << ',' << cur_cmd[2] << ',' << cur_cmd[3]
           << ',' << cur_meas[0]<< ',' << cur_meas[1]<< ',' << cur_meas[2]<< ',' << cur_meas[3]
           << '\n';
    }

    // 10. Log periódico (~1 s)
    if (++log_cnt_ % static_cast<int>(std::lround(loop_rate_hz_)) == 0) {
      if (csv_.is_open()) csv_.flush();
      RCLCPP_INFO(get_logger(),
        "t=%.1fs  q=[%.3f %.3f %.3f %.3f]  |e|=%.3f  i_cmd=[%d %d %d %d]  i_meas=[%d %d %d %d]",
        t, q(0), q(1), q(2), q(3), (q - ref.q).norm(),
        cur_cmd[0], cur_cmd[1], cur_cmd[2], cur_cmd[3],
        cur_meas[0], cur_meas[1], cur_meas[2], cur_meas[3]);
    }

    // 11. Overrun: el ciclo (bus + control) no cabe en Ts
    const double tick_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - tick_t0).count();
    if (tick_ms > 1e3 * Ts_ && ++overrun_cnt_ % 50 == 1) {
      RCLCPP_WARN(get_logger(), "Overrun del lazo: tick=%.2f ms > Ts=%.1f ms (n=%d)",
        tick_ms, 1e3 * Ts_, overrun_cnt_);
    }
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Miembros
  // ─────────────────────────────────────────────────────────────────────────

  omx::RobotModel model_;

  std::string port_name_;
  bool   open_port_;
  int    log_id_;
  double t_run_, gain_scale_;
  Vec4   kp_, kd_;

  omx::Calibration calib_;   // ceros, signos (YAML del motor)
  omx::HwLimits    limits_;  // limites articulares y de corriente (YAML del motor)

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

  // Auto-carga del YAML de motor SOLO por su bloque de hardware (ceros, signos,
  // límites): motor_alpha/Fv/Fc/I_offset no se declaran en este nodo y se
  // ignoran. Va al final → tiene prioridad sobre los -p del CLI.
  const rclcpp::NodeOptions opts = omx::motorParamsNodeOptions(argc, argv, "hw_alpha_ident_node");

  try {
    auto node = std::make_shared<HWAlphaIdentNode>(opts);
    if (node->dry_run()) node->preflight();
    else                 rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("hw_alpha_ident_node"), "Excepcion: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
