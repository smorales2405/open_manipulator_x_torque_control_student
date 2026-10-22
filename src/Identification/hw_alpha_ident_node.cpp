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
 * la fricción que el PD vence en cada inversión.
 *
 * Trayectoria: la misma act1 del laboratorio de FL (senos de 1 rad/s en J1–J3,
 * J4 fijo en π/4), probada en el robot real dentro de límites y sin colisiones.
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

#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include "dynamixel_sdk/dynamixel_sdk.h"

#ifndef PACKAGE_DATA_DIR
#define PACKAGE_DATA_DIR "."
#endif
#ifndef PACKAGE_URDF_DIR
#define PACKAGE_URDF_DIR "."
#endif
#ifndef PACKAGE_CONFIG_DIR
#define PACKAGE_CONFIG_DIR "."
#endif

using namespace std::chrono_literals;

// ============================================================
// Constantes Dynamixel / conversión de unidades
// ============================================================

static constexpr double PI = 3.14159265358979323846;
static constexpr int    NUM_JOINTS = 4;

static const std::array<uint8_t, NUM_JOINTS> DXL_ID = {11, 12, 13, 14};
static constexpr int    BAUDRATE         = 1000000;
static constexpr double PROTOCOL_VERSION = 2.0;

static constexpr uint16_t ADDR_OPERATING_MODE   = 11;
static constexpr uint16_t ADDR_CURRENT_LIMIT    = 38;
static constexpr uint16_t ADDR_TORQUE_ENABLE    = 64;
static constexpr uint16_t ADDR_GOAL_CURRENT     = 102;
static constexpr uint16_t ADDR_PRESENT_CURRENT  = 126;
static constexpr uint16_t ADDR_PRESENT_VELOCITY = 128;
static constexpr uint16_t ADDR_PRESENT_POSITION = 132;

static constexpr uint16_t LEN_GOAL_CURRENT     = 2;
static constexpr uint16_t LEN_PRESENT_CURRENT  = 2;
static constexpr uint16_t LEN_PRESENT_VELOCITY = 4;
static constexpr uint16_t LEN_PRESENT_POSITION = 4;

// Bloque contiguo current(126,2)+velocity(128,4)+position(132,4): una sola
// GroupSyncRead por ciclo.
static constexpr uint16_t ADDR_STATE_BLOCK = ADDR_PRESENT_CURRENT;
static constexpr uint16_t LEN_STATE_BLOCK  = 10;

static constexpr uint8_t CURRENT_CONTROL_MODE = 0;
static constexpr uint8_t TORQUE_ENABLE_VAL    = 1;
static constexpr uint8_t TORQUE_DISABLE_VAL   = 0;

static constexpr double POS_UNIT_RAD   = 2.0 * PI / 4096.0;
static constexpr double VEL_UNIT_RAD_S = 0.229 * 2.0 * PI / 60.0;
static constexpr double CURRENT_UNIT_A = 0.00269;

// Conversión NOMINAL fija del ensayo: kt del datasheet XM430-W350 (stall
// 4.1 N·m / 2.3 A a 12 V) → ALPHA_NOM ≈ 208.5 ticks/N·m. Sin Fv/Fc/I_offset.
static constexpr double KT_DATASHEET_NM_A = 1.7826;
static constexpr double ALPHA_NOM = 1.0 / (KT_DATASHEET_NM_A * CURRENT_UNIT_A);

// JOINT_ZERO_TICK, ENCODER_SIGN, CURRENT_SIGN, JOINT_LOWER/UPPER,
// CURRENT_LIMIT_REGISTER, CURRENT_CMD_LIMIT, CURRENT_MEASURED_PEAK
// → cargados desde config/motorXM430W350T_params.yaml como parámetros ROS 2.

static constexpr double RAMP_TIME_S   = 3.0;
static constexpr double RETURN_TIME_S = 3.0;   // duración del retorno quíntico a q_inicial
static constexpr double HOLD_TIME_S   = 0.5;   // pausa en reposo antes de cortar corriente
static constexpr double TAU_MAX       = 1.2;   // [N·m] saturación del torque comandado
static constexpr double MAX_TRACK_ERR = 0.5;   // [rad] error de seguimiento → parada
                                               // (signos invertidos, colisión, atasco)

using Vec4 = Eigen::Matrix<double, NUM_JOINTS, 1>;

// ============================================================
// Utilidades de conversión (idénticas a hw_fl_control_node)
// ============================================================

static int32_t toSigned32(uint32_t v)
{
  if (v > 0x7FFFFFFFu) return -static_cast<int32_t>(0xFFFFFFFFu - v + 1u);
  return static_cast<int32_t>(v);
}

static int16_t toSigned16(uint32_t v)
{
  const uint16_t w = static_cast<uint16_t>(v & 0xFFFFu);
  if (w > 0x7FFFu) return -static_cast<int16_t>(0xFFFFu - w + 1u);
  return static_cast<int16_t>(w);
}

static int32_t wrappedTickDiff(int32_t raw, int32_t zero)
{
  int32_t d = raw - zero;
  while (d >  2048) d -= 4096;
  while (d < -2048) d += 4096;
  return d;
}

static void currentToBytes(int16_t cur, uint8_t p[2])
{
  const uint16_t w = static_cast<uint16_t>(cur);
  p[0] = DXL_LOBYTE(w);
  p[1] = DXL_HIBYTE(w);
}

static int16_t clampCurrent(double x, int16_t lim)
{
  return static_cast<int16_t>(std::lround(
    std::min(std::max(x, -static_cast<double>(lim)), static_cast<double>(lim))));
}

// ============================================================
// Trayectoria deseada (articular act1, idéntica a hw_fl_control_node)
// ============================================================

struct Reference { Vec4 q, dq, ddq; };

static Reference desiredTrajectory(double t)
{
  const double w = 1.0;
  Reference r;
  r.q   << (PI/4.0)*std::sin(w*t),      -0.5 + 0.5*std::sin(w*t),    0.3 - 0.5*std::sin(w*t),   (PI/4.0);
  r.dq  << (PI/4.0)*w*std::cos(w*t),     0.5*w*std::cos(w*t),        -0.5*w*std::cos(w*t),         0.0;
  r.ddq << -(PI/4.0)*w*w*std::sin(w*t), -0.5*w*w*std::sin(w*t),      0.5*w*w*std::sin(w*t),        0.0;
  return r;
}

// Polinomio quíntico general: interpola (q,dq,ddq) desde (q0,v0,a0) en t=0
// hasta (qf,vf,af) en t=T.
static Reference quinticBlend(double t, double T,
                              const Vec4& q0, const Vec4& v0, const Vec4& a0,
                              const Vec4& qf, const Vec4& vf, const Vec4& af)
{
  if (T <= 0.0) { Reference r; r.q = qf; r.dq = vf; r.ddq = af; return r; }

  const double T2=T*T, T3=T2*T, T4=T3*T, T5=T4*T;
  const Vec4 c0 = q0;
  const Vec4 c1 = v0;
  const Vec4 c2 = 0.5 * a0;
  const Vec4 c3 = (20.0*(qf-q0) - (8.0*vf+12.0*v0)*T - (3.0*a0-af)*T2) / (2.0*T3);
  const Vec4 c4 = (30.0*(q0-qf) + (14.0*vf+16.0*v0)*T + (3.0*a0-2.0*af)*T2) / (2.0*T4);
  const Vec4 c5 = (12.0*(qf-q0) - (6.0*vf+6.0*v0)*T - (a0-af)*T2) / (2.0*T5);

  const double t2=t*t, t3=t2*t, t4=t3*t, t5=t4*t;
  Reference r;
  r.q   = c0 + c1*t  + c2*t2  + c3*t3  + c4*t4  + c5*t5;
  r.dq  = c1 + 2.0*c2*t + 3.0*c3*t2 + 4.0*c4*t3 + 5.0*c5*t4;
  r.ddq = 2.0*c2 + 6.0*c3*t + 12.0*c4*t2 + 20.0*c5*t3;
  return r;
}

// Referencia completa: rampa desde el reposo en q0 → trayectoria periódica →
// retorno en reposo a q0 → pausa. t_run <= 0 desactiva el retorno.
static Reference referenceAt(double t, double t_run, const Vec4& q0)
{
  if (t < RAMP_TIME_S) {
    const Reference target = desiredTrajectory(RAMP_TIME_S);
    return quinticBlend(t, RAMP_TIME_S, q0, Vec4::Zero(), Vec4::Zero(),
                        target.q, target.dq, target.ddq);
  }
  if (t_run <= 0.0 || t < t_run) return desiredTrajectory(t);
  if (t < t_run + RETURN_TIME_S) {
    const Reference start = desiredTrajectory(t_run);
    return quinticBlend(t - t_run, RETURN_TIME_S, start.q, start.dq, start.ddq,
                        q0, Vec4::Zero(), Vec4::Zero());
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
    hw_active_(false), q_initial_captured_(false)
  {
    // ── Parámetros ──────────────────────────────────────────────────────────
    using dvec = std::vector<double>;
    using ivec = std::vector<int64_t>;
    this->declare_parameter<std::string>("port_name",  "/dev/ttyUSB0");
    this->declare_parameter<bool>       ("open_port",  true);
    this->declare_parameter<int>        ("log_id",     30);
    this->declare_parameter<double>     ("t_run",      40.0);
    this->declare_parameter<double>     ("gain_scale", 1.0);
    // Rigidez/amortiguamiento ≈ M_ii(q)·kp y M_ii(q)·kd del FL (kp=[400 400 600
    // 3000], kd=[28 28 34 77]) con la inercia media del URDF sobre act1:
    // misma banda (ωn ≈ 20–50 rad/s) y ζ ≈ 0.5–1.0 en todo el recorrido.
    this->declare_parameter<dvec>("kp", dvec{2.5,  5.0,  5.0,  3.0 });
    this->declare_parameter<dvec>("kd", dvec{0.17, 0.33, 0.28, 0.08});
    this->declare_parameter<double>("ab_alpha", 0.2);
    this->declare_parameter<double>("ab_beta",  0.02);
    this->declare_parameter<double>("loop_rate_hz", 200.0);

    this->declare_parameter<ivec>  ("joint_zero_tick",        ivec{2048, 2048, 2048, 2048});
    this->declare_parameter<dvec>  ("encoder_sign",           dvec{+1.0, +1.0, +1.0, +1.0});
    this->declare_parameter<dvec>  ("current_sign",           dvec{+1.0, +1.0, +1.0, +1.0});
    this->declare_parameter<dvec>  ("joint_lower",            dvec{-2.356194, -1.919862, -1.919862, -1.8});
    this->declare_parameter<dvec>  ("joint_upper",            dvec{+2.356194, +1.745329, +1.570796, +2.1});
    this->declare_parameter<int>   ("current_limit_register", 350);
    this->declare_parameter<ivec>  ("current_cmd_limit",      ivec{257, 257, 257, 257});
    this->declare_parameter<int>   ("current_measured_peak",  313);

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

    {
      const auto zt = get_parameter("joint_zero_tick").as_integer_array();
      for (int i = 0; i < NUM_JOINTS; ++i) joint_zero_tick_[i] = static_cast<int32_t>(zt[i]);
    }
    encoder_sign_ = load_vec4("encoder_sign");
    current_sign_ = load_vec4("current_sign");
    joint_lower_  = load_vec4("joint_lower");
    joint_upper_  = load_vec4("joint_upper");
    current_limit_register_ = static_cast<uint16_t>(get_parameter("current_limit_register").as_int());
    {
      const auto cl = get_parameter("current_cmd_limit").as_integer_array();
      for (int i = 0; i < NUM_JOINTS; ++i) current_cmd_limit_[i] = static_cast<int16_t>(cl[i]);
    }
    current_measured_peak_ = static_cast<int16_t>(get_parameter("current_measured_peak").as_int());

    RCLCPP_INFO(get_logger(), "puerto=%s  open_port=%d  t_run=%.1fs  gain=%.2f  id=%d",
      port_name_.c_str(), open_port_ ? 1 : 0, t_run_, gain_scale_, log_id_);
    RCLCPP_INFO(get_logger(),
      "Conversión FIJA del ensayo: α=%.2f ticks/N·m (nominal), sin Fv/Fc/I_offset "
      "(el YAML de motor no se usa)", ALPHA_NOM);
    RCLCPP_INFO(get_logger(), "PD+G: Kp=[%.2f %.2f %.2f %.2f] N·m/rad  Kd=[%.3f %.3f %.3f %.3f] N·m·s/rad",
      kp_(0), kp_(1), kp_(2), kp_(3), kd_(0), kd_(1), kd_(2), kd_(3));
    RCLCPP_INFO(get_logger(),
      "hw: zero=[%d %d %d %d]  enc_sign=[%.0f %.0f %.0f %.0f]  cur_sign=[%.0f %.0f %.0f %.0f]",
      joint_zero_tick_[0], joint_zero_tick_[1], joint_zero_tick_[2], joint_zero_tick_[3],
      encoder_sign_(0), encoder_sign_(1), encoder_sign_(2), encoder_sign_(3),
      current_sign_(0), current_sign_(1), current_sign_(2), current_sign_(3));

    if (t_run_ <= RAMP_TIME_S + 10.0) {
      RCLCPP_WARN(get_logger(), "t_run=%.1f s es corto: identify_alpha.py descarta los "
        "primeros 4 s; use t_run ≥ 30 s.", t_run_);
    }

    // ── Pinocchio (solo G(q)) ────────────────────────────────────────────────
    const std::string urdf = std::string(PACKAGE_URDF_DIR) + "/open_manipulator_x.urdf";
    try {
      pinocchio::urdf::buildModel(urdf, model_);
    } catch (const std::exception& e) {
      RCLCPP_FATAL(get_logger(), "Pinocchio URDF: %s", e.what());
      throw;
    }
    data_ = pinocchio::Data(model_);

    // Dry-run: sin hardware, sin CSV, sin timer (main llama a preflight()).
    if (!open_port_) return;

    // ── CSV ──────────────────────────────────────────────────────────────────
    const std::string dir = std::string(PACKAGE_DATA_DIR) + "/identification/alpha";
    std::filesystem::create_directories(dir);
    csv_path_ = dir + "/hw_alpha_ident_" + std::to_string(log_id_) + ".csv";
    csv_.open(csv_path_);
    if (csv_.is_open()) {
      csv_ << "t,q1,q2,q3,q4,dq1,dq2,dq3,dq4,dq1_filt,dq2_filt,dq3_filt,dq4_filt,"
              "q1_des,q2_des,q3_des,q4_des,dq1_des,dq2_des,dq3_des,dq4_des,"
              "tau1,tau2,tau3,tau4,tau_g1,tau_g2,tau_g3,tau_g4,"
              "curr_cmd1,curr_cmd2,curr_cmd3,curr_cmd4,"
              "curr_meas1,curr_meas2,curr_meas3,curr_meas4\n";
      RCLCPP_INFO(get_logger(), "CSV: %s", csv_path_.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "No se pudo crear CSV: %s", csv_path_.c_str());
    }

    js_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/hw/joint_states", 10);

    // ── SDK ──────────────────────────────────────────────────────────────────
    if (!init_hardware()) {
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
    shutdown_hardware();
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
        if (ref.q(i) < joint_lower_(i) + 0.02 || ref.q(i) > joint_upper_(i) - 0.02) limits_ok = false;
      tau_pk = tau_pk.cwiseMax(gravity(ref.q).cwiseAbs());
    }
    RCLCPP_INFO(get_logger(), "══ DRY-RUN (open_port=false): no se abre el puerto ni se escribe CSV ══");
    RCLCPP_INFO(get_logger(), "Duración: %.1f s de ensayo (%d filas de CSV a %.0f Hz) + %.1f s de retorno",
      t_run_, static_cast<int>(t_run_ * loop_rate_hz_), loop_rate_hz_, RETURN_TIME_S + HOLD_TIME_S);
    for (int i = 0; i < NUM_JOINTS; ++i) {
      const double I_pk = ALPHA_NOM * tau_pk(i);
      RCLCPP_INFO(get_logger(), "J%d: q ∈ [%+.3f, %+.3f] rad (límites [%+.3f, %+.3f])  "
        "|G|max=%.3f N·m → |I_cmd| ≈ %.0f ticks (límite %d)%s",
        i + 1, q_min(i), q_max(i), joint_lower_(i), joint_upper_(i), tau_pk(i), I_pk,
        static_cast<int>(current_cmd_limit_[i]),
        I_pk > 0.8 * current_cmd_limit_[i] ? "  ← margen de corriente bajo" : "");
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
  //  Hardware
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

    for (const auto id : DXL_ID)
      dxl_write1(id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE_VAL, "pre-disable");

    for (const auto id : DXL_ID) {
      if (!dxl_write1(id, ADDR_OPERATING_MODE, CURRENT_CONTROL_MODE, "set mode") ||
          !dxl_write2(id, ADDR_CURRENT_LIMIT, current_limit_register_, "set limit") ||
          !dxl_write1(id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE_VAL, "enable torque")) {
        port_handler_->closePort();
        return false;
      }
      RCLCPP_INFO(get_logger(), "DXL ID %d listo", static_cast<int>(id));
    }

    grp_read_ = std::make_unique<dynamixel::GroupSyncRead>(
      port_handler_, packet_handler_, ADDR_STATE_BLOCK, LEN_STATE_BLOCK);
    grp_wcur_ = std::make_unique<dynamixel::GroupSyncWrite>(
      port_handler_, packet_handler_, ADDR_GOAL_CURRENT, LEN_GOAL_CURRENT);

    for (const auto id : DXL_ID)
      grp_read_->addParam(id);

    hw_active_ = true;
    RCLCPP_INFO(get_logger(), "Hardware inicializado en %s", port_name_.c_str());
    return true;
  }

  void shutdown_hardware()
  {
    if (!hw_active_) return;
    hw_active_ = false;

    if (grp_wcur_) {
      const std::array<int16_t, NUM_JOINTS> zero = {0, 0, 0, 0};
      send_currents(zero);
      rclcpp::sleep_for(std::chrono::milliseconds(20));
    }
    for (const auto id : DXL_ID)
      dxl_write1(id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE_VAL, "shutdown disable");

    if (port_handler_) {
      port_handler_->closePort();
      RCLCPP_INFO(get_logger(), "Puerto cerrado.");
    }
  }

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

  bool read_state(Vec4& q, Vec4& dq, std::array<int16_t, NUM_JOINTS>& cur)
  {
    if (grp_read_->txRxPacket() != COMM_SUCCESS) {
      RCLCPP_ERROR(get_logger(), "SyncRead fallo");
      return false;
    }
    for (int i = 0; i < NUM_JOINTS; ++i) {
      const uint8_t id = DXL_ID[i];
      if (!grp_read_->isAvailable(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION) ||
          !grp_read_->isAvailable(id, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY) ||
          !grp_read_->isAvailable(id, ADDR_PRESENT_CURRENT,  LEN_PRESENT_CURRENT)) {
        RCLCPP_ERROR(get_logger(), "[ID %d] dato no disponible", id);
        return false;
      }
      const int32_t rp = toSigned32(static_cast<uint32_t>(
        grp_read_->getData(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)));
      const int32_t rv = toSigned32(static_cast<uint32_t>(
        grp_read_->getData(id, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY)));
      const int16_t rc = toSigned16(static_cast<uint32_t>(
        grp_read_->getData(id, ADDR_PRESENT_CURRENT,  LEN_PRESENT_CURRENT)));

      q(i)   = encoder_sign_(i) * static_cast<double>(wrappedTickDiff(rp, joint_zero_tick_[i])) * POS_UNIT_RAD;
      dq(i)  = encoder_sign_(i) * static_cast<double>(rv) * VEL_UNIT_RAD_S;
      cur[i] = rc;
    }
    return true;
  }

  bool send_currents(const std::array<int16_t, NUM_JOINTS>& cmd)
  {
    uint8_t p[NUM_JOINTS][2];
    for (int i = 0; i < NUM_JOINTS; ++i) {
      currentToBytes(cmd[i], p[i]);
      grp_wcur_->addParam(DXL_ID[i], p[i]);
    }
    const int r = grp_wcur_->txPacket();
    grp_wcur_->clearParam();
    if (r != COMM_SUCCESS) {
      RCLCPP_ERROR(get_logger(), "SyncWrite fallo");
      return false;
    }
    return true;
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Ley de control: PD articular + G(q)
  // ─────────────────────────────────────────────────────────────────────────

  Vec4 gravity(const Vec4& q)
  {
    Eigen::VectorXd q_pin = Eigen::VectorXd::Zero(model_.nq);
    q_pin.head(NUM_JOINTS) = q;
    return pinocchio::computeGeneralizedGravity(model_, data_, q_pin).head(NUM_JOINTS);
  }

  // Conversión fija del ensayo: I = ALPHA_NOM·tau, sin fricción ni offset.
  std::array<int16_t, NUM_JOINTS> torque_to_current(const Vec4& tau)
  {
    std::array<int16_t, NUM_JOINTS> cmd{};
    for (int i = 0; i < NUM_JOINTS; ++i)
      cmd[i] = clampCurrent(current_sign_(i) * encoder_sign_(i) * ALPHA_NOM * tau(i),
                            current_cmd_limit_[i]);
    return cmd;
  }

  void emergency_stop(const std::string& reason)
  {
    RCLCPP_ERROR(get_logger(), "PARADA: %s", reason.c_str());
    timer_->cancel();
    shutdown_hardware();
    if (csv_.is_open()) { csv_.flush(); csv_.close(); }
    rclcpp::shutdown();
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Callback del timer (loop_rate_hz)
  // ─────────────────────────────────────────────────────────────────────────

  void tick()
  {
    if (!hw_active_) return;

    const auto tick_t0 = std::chrono::steady_clock::now();
    const double t = std::chrono::duration<double>(
      std::chrono::high_resolution_clock::now() - start_time_).count();

    // 1. Lectura de estado
    Vec4 q, dq;
    std::array<int16_t, NUM_JOINTS> cur_meas{};
    if (!read_state(q, dq, cur_meas)) {
      emergency_stop("SyncRead fallido");
      return;
    }

    if (t_run_ > 0.0 && t >= t_run_ + RETURN_TIME_S + HOLD_TIME_S) {
      RCLCPP_INFO(get_logger(), "Retorno completado: el robot quedó a %.3f rad de q_inicial. "
        "Torque off (t=%.1f s).", (q - q_initial_).norm(), t);
      RCLCPP_INFO(get_logger(), "Analizar: python3 src/Identification/identify_alpha.py %d",
        log_id_);
      timer_->cancel();
      shutdown_hardware();
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
      if (ref.q(i) < joint_lower_(i) + 0.02 || ref.q(i) > joint_upper_(i) - 0.02) {
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
    const Vec4 tau = tau_unsat.cwiseMin(TAU_MAX).cwiseMax(-TAU_MAX);
    const auto cur_cmd = torque_to_current(tau);

    // 6. Escribir corriente
    if (!send_currents(cur_cmd)) {
      emergency_stop("SyncWrite fallido");
      return;
    }

    // 7. Verificar corriente medida (seguridad)
    for (int i = 0; i < NUM_JOINTS; ++i) {
      if (std::abs(cur_meas[i]) > current_measured_peak_) {
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

  pinocchio::Model model_;
  pinocchio::Data  data_;

  std::string port_name_;
  bool   open_port_;
  int    log_id_;
  double t_run_, gain_scale_;
  Vec4   kp_, kd_;

  std::array<int32_t, NUM_JOINTS> joint_zero_tick_;
  Vec4     encoder_sign_, current_sign_;
  Vec4     joint_lower_, joint_upper_;
  uint16_t current_limit_register_;
  std::array<int16_t, NUM_JOINTS> current_cmd_limit_;
  int16_t  current_measured_peak_;

  double ab_alpha_, ab_beta_;
  double loop_rate_hz_{200.0};
  double Ts_{0.005};
  Vec4   q_hat_, dq_hat_;
  bool   ab_initialized_{false};

  dynamixel::PortHandler*   port_handler_{nullptr};
  dynamixel::PacketHandler* packet_handler_{nullptr};
  std::unique_ptr<dynamixel::GroupSyncRead>  grp_read_;
  std::unique_ptr<dynamixel::GroupSyncWrite> grp_wcur_;

  bool hw_active_;
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
  rclcpp::NodeOptions opts;
  const std::string cfg = std::string(PACKAGE_CONFIG_DIR) + "/motorXM430W350T_params.yaml";
  if (std::filesystem::exists(cfg)) {
    std::vector<std::string> args = {"--ros-args"};
    bool in_ros_args = false;
    for (int i = 1; i < argc; ++i) {
      const std::string a(argv[i]);
      if (a == "--ros-args") { in_ros_args = true; continue; }
      if (in_ros_args) args.push_back(a);
    }
    args.push_back("--params-file");
    args.push_back(cfg);
    opts.arguments(args);
    opts.use_global_arguments(false);
    RCLCPP_INFO(rclcpp::get_logger("hw_alpha_ident_node"),
      "Bloque de hardware de %s auto-cargado", cfg.c_str());
  } else {
    RCLCPP_WARN(rclcpp::get_logger("hw_alpha_ident_node"),
      "motorXM430W350T_params no encontrado: %s — usando defaults del código.", cfg.c_str());
  }

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
