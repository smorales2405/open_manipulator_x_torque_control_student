/*
 * hw_SMC_joint_node_student.cpp
 * Control por Modo Deslizante en espacio articular — OpenMANIPULATOR-X
 * hardware real vía Dynamixel SDK directo, sin ros2_control.
 * [Archivo base para estudiantes — SMC, implementacion]
 *
 * ──────────────────────────────────────────────────────────────────────────
 * SECCIONES A COMPLETAR:
 *   [1] Trayectoria articular de referencia  →  desiredTrajectory()
 *   [2] Funciones de conmutacion             →  rho_scalar()
 *   [3] Ganancias SMC                        →  LAMBDA_Q, K_V, K_S
 *   [4] Ley SMC articular                    →  compute_torque()
 * El resto (hardware Dynamixel, filtro alpha-beta, transicion quintica,
 * conversion torque→corriente, guardas de seguridad, CSV) es
 * INFRAESTRUCTURA: no modificar.
 * ──────────────────────────────────────────────────────────────────────────
 *
 * Superficie deslizante articular:
 *   e_q  = q  - q_d,   e_dq = dq - dq_d
 *   s_q  = e_dq + Lambda_q * e_q
 *
 * Ley de control SMC (ley de alcance exponencial + conmutacion):
 *   v_q  = ddq_d - Lambda_q*e_dq - K_v*s_q - K_s*rho(s_q)
 *   tau  = M(q)*v_q + Phi(q,dq)          [RNEA de Pinocchio]
 *   tau_sat = clamp(tau, -tau_max, tau_max)
 *
 * NOTA sobre la friccion: a diferencia de los nodos gz (feedforward del URDF
 * a nivel de torque), en hardware la friccion REAL se compensa en el modelo
 * torque→corriente identificado (Fv, Fc en ticks, con Fc alimentado por la
 * velocidad DESEADA). NO agregar tau_fric del URDF aqui: seria doble
 * compensacion. El residual de friccion es la incertidumbre acotada que el
 * termino K_s*rho(s) debe dominar.
 *
 * Funciones de conmutacion (elemento a elemento):
 *   "sign" -> rho(s) = sign(s)
 *   "sat"  -> rho(s) = sat(s/phi)     phi: capa limite [rad/s]
 *
 * Trayectoria articular de referencia (omega = 1.0 rad/s, identica a gz):
 *   q_d = [ (pi/4)*sin(w*t'), -0.45+0.5*sin(w*t'), 0.35-0.5*sin(w*t'),
 *           pi/4+0.25*sin(w*t') ],   t' = t - T_TRANS
 * Transicion inicial: quintica generalizada [0, T_TRANS) desde la pose
 * medida hasta q_d(0) con empalme C2 (dq_d(0) = B*w != 0).
 *
 * Fases temporales completas (ver constantes T_TRANS/RETURN_TIME_S/HOLD_TIME_S):
 *   [0, T_TRANS)                                  transicion quintica a q_d(0)
 *   [T_TRANS, T_TRANS+t_run)                      seguimiento periodico
 *   [T_TRANS+t_run, T_TRANS+t_run+RETURN_TIME_S)  retorno quintico a q_inicial
 *   [...+RETURN_TIME_S, ...+HOLD_TIME_S)          pausa en reposo (asienta)
 *   >= ...+RETURN_TIME_S+HOLD_TIME_S              corte de corriente y fin
 * El retorno evita que el brazo caiga por gravedad al recibir torque cero
 * de golpe en medio de la trayectoria (t_run<=0 desactiva el retorno: corre
 * indefinidamente). El CSV registra la transicion inicial + el seguimiento
 * (en hardware las fallas suelen ocurrir en la transicion); plots_SMC_joint.m
 * en modo 'real' descarta t < T_TRANS para que las metricas sean solo de
 * seguimiento. El retorno y la pausa final NO se guardan en el CSV.
 *
 * Parametros ROS 2 (--ros-args -p nombre:=valor):
 *   port_name          [string]  "/dev/ttyUSB0"
 *   gain_scale         [double]  1.0    (escala lineal de K_V y K_S; Lambda fija)
 *   t_run              [double]  20.0   (duracion del SEGUIMIENTO [s];
 *                                        total = T_TRANS + t_run)
 *   test_num           [int]     1      (CSV: hw_smc_joint_<rho_func>_<test_num>.csv)
 *   rho_func           [string]  "sat"  (funcion de conmutacion: "sign" | "sat")
 *   phi                [double]  0.3    (capa limite para sat(s/phi) [rad/s])
 *   ab_alpha           [double]  0.2    (filtro α-β: ganancia de posicion)
 *   ab_beta            [double]  0.02   (filtro α-β: ganancia de velocidad)
 *   friction_fc_scale  [double]  0.95   (fraccion de Fc compensada; Fc usa dq DESEADA)
 *   loop_rate_hz       [double]  200.0  (frecuencia del lazo [Hz], acotada [50,400])
 *
 * Parametros del modelo identificado (auto-cargados desde
 * config/motorXM430W350T_params.yaml): motor_alpha, motor_Fv, motor_Fc,
 * motor_I_offset, motor_epsilon_friction, joint_zero_tick, encoder_sign,
 * current_sign, joint_lower/upper, current_*_limit, tau_max.
 *
 * CSV: data/smc/real/act1/hw_smc_joint_<rho_func>_<test_num>.csv
 * Columnas (compatibles con plots_SMC_joint.m, mode='real'):
 *   t, q1..q4, dq1..dq4, q1_des..q4_des, dq1_des..dq4_des, s1..s4,
 *   tau1..tau4, dq1_filt..dq4_filt, curr_cmd1..4, curr_meas1..4
 * (Sat% se calcula en MATLAB desde tau con el criterio >= 0.99*tau_max.)
 *
 * Publisher: /hw/joint_states (sensor_msgs/JointState) — solo monitoreo
 *
 * Ejemplos de ejecucion (primer run SIEMPRE con gain_scale:=0.5):
 *   ros2 run open_manipulator_x_torque_control hw_smc_joint_node_student \
 *     --ros-args -p gain_scale:=0.5 -p rho_func:=sat -p test_num:=1 -p t_run:=30.0
 *
 *   ros2 run open_manipulator_x_torque_control hw_smc_joint_node_student \
 *     --ros-args -p rho_func:=sat -p phi:=0.3 -p test_num:=2 -p t_run:=30.0
 *
 * ADVERTENCIA: No ejecutar junto a hardware.launch.py ni ningun proceso
 * que acceda a /dev/ttyUSB0 (ros2_control_node, dynamixel_hardware_interface).
 */

#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <vector>
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

static constexpr double PI = 3.14159265358979323846;
static constexpr int    NUM_JOINTS = 4;

// Calibracion, limites y modelo del motor: omx::declareHwConfig
// (config/motorXM430W350T_params.yaml como parametros ROS 2).

// Duracion de la transicion quintica desde la pose medida hasta q_d(0)
static constexpr double T_TRANS = 3.0;   // [s]
static constexpr double RETURN_TIME_S = 3.0;   // duracion del retorno quintico a q_inicial
static constexpr double HOLD_TIME_S   = 0.5;   // pausa en reposo antes de cortar corriente

using Vec4 = Eigen::Matrix<double, NUM_JOINTS, 1>;

// ═══════════════════════════════════════════════════════════════════════════
//  [SECCION 3] GANANCIAS SMC — COMPLETAR
//  Ajustar los valores para cada articulacion [joint1, joint2, joint3, joint4]
//
//  En HARDWARE la stiction real exige rigidez varias veces mayor que en
//  Gazebo (en joint4, M44 ~ 0.001 kg·m² convierte aceleraciones en
//  mili-torques: las ganancias de sim dejan la muneca pegada). Dentro de la
//  capa el SMC equivale a un PD con:
//    kp_eq = Lambda*(K_V + K_S/phi)   kd_eq = Lambda + K_V + K_S/phi
//  Apuntar a la rigidez validada por FL: kp_eq ~ [400, 400, 600, 3000].
//
//  Rangos recomendados (con phi = 0.3):
//    LAMBDA_Q : 15 – 45   [1/s]
//    K_V      : 10 – 60
//    K_S      : 1  – 8    [rad/s²]
//  Primer run SIEMPRE con gain_scale:=0.5.
// ═══════════════════════════════════════════════════════════════════════════
static const Vec4 LAMBDA_Q = (Vec4() << 0.0, 0.0, 0.0, 0.0).finished();  // COMPLETAR
static const Vec4 K_V      = (Vec4() << 0.0, 0.0, 0.0, 0.0).finished();  // COMPLETAR
static const Vec4 K_S      = (Vec4() << 0.0, 0.0, 0.0, 0.0).finished();  // COMPLETAR
// ═══════════════════════════════════════════════════════════════════════════

// ============================================================
// Trayectoria articular de referencia (identica a gz_SMC_joint_node)
// ============================================================

using omx::Reference;

// ─────────────────────────────────────────────────────────────────────────────
//  [SECCION 1] Trayectoria articular de referencia — COMPLETAR
//
//  Definir la trayectoria para cada articulacion de la forma:
//    qi_d(t)   = Ai + Bi * sin(w*t)
//    dqi_d(t)  = Bi * w * cos(w*t)
//    ddqi_d(t) = -Bi * w^2 * sin(w*t)
//
//  Usar w = 1.0 rad/s (no modificar). Misma trayectoria que en Gazebo.
// ─────────────────────────────────────────────────────────────────────────────
static Reference desiredTrajectory(double t)
{
  Reference r;

  (void)t;
  const double w = 1.0;  // [rad/s] — no modificar
  (void)w;

  // COMPLETAR: posicion deseada qi_d(t) = Ai + Bi*sin(w*t)
  r.q <<
      0.0,   // joint1: COMPLETAR
      0.0,   // joint2: COMPLETAR
      0.0,   // joint3: COMPLETAR
      0.0;   // joint4: COMPLETAR

  // COMPLETAR: velocidad deseada dqi_d(t) = Bi*w*cos(w*t)
  r.dq <<
      0.0,   // joint1: COMPLETAR
      0.0,   // joint2: COMPLETAR
      0.0,   // joint3: COMPLETAR
      0.0;   // joint4: COMPLETAR

  // COMPLETAR: aceleracion deseada ddqi_d(t) = -Bi*w^2*sin(w*t)
  r.ddq <<
      0.0,   // joint1: COMPLETAR
      0.0,   // joint2: COMPLETAR
      0.0,   // joint3: COMPLETAR
      0.0;   // joint4: COMPLETAR

  return r;
}

// ─────────────────────────────────────────────────────────────────────────────
//  [SECCION 2] Funciones de conmutacion — COMPLETAR
//
//  Implementar rho(s) para cada tipo:
//    sign : rho(s) = +1 si s > 0,  -1 si s < 0,  0 si s == 0
//    sat  : rho(s) = clamp(s / phi, -1, 1)        (phi: capa limite)
// ─────────────────────────────────────────────────────────────────────────────
enum class RhoFunc { SIGN, SAT };

static double rho_scalar(double s, RhoFunc func, double phi)
{
  (void)s; (void)phi;

  switch (func) {
    case RhoFunc::SIGN:
      return 0.0;  // COMPLETAR: sign(s)
    case RhoFunc::SAT:
      return 0.0;  // COMPLETAR: sat(s/phi) = clamp(s/phi, -1, 1)
    default:
      return 0.0;
  }
}

// Aplica rho_scalar elemento a elemento sobre el vector s (no modificar)
[[maybe_unused]] static Vec4 rho_vec(const Vec4 & s, RhoFunc func, double phi)
{
  Vec4 r;
  for (int i = 0; i < NUM_JOINTS; ++i) {
    r[i] = rho_scalar(s[i], func, phi);
  }
  return r;
}

// ============================================================
// Nodo ROS 2
// ============================================================

class HWSMCJointNode : public rclcpp::Node
{
public:
  explicit HWSMCJointNode(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions())
  : Node("hw_smc_joint_node", opts), q_initial_captured_(false)
  {
    // ── Parámetros ──────────────────────────────────────────────────────────
    this->declare_parameter<std::string>("port_name",     "/dev/ttyUSB0");
    this->declare_parameter<double>     ("gain_scale",     1.0);
    this->declare_parameter<double>     ("t_run",          20.0);
    this->declare_parameter<int>        ("test_num",       1);
    this->declare_parameter<std::string>("rho_func",       "sat");
    this->declare_parameter<double>     ("phi",            0.3);

    this->declare_parameter<double>("ab_alpha", 0.2);
    this->declare_parameter<double>("ab_beta",  0.02);
    this->declare_parameter<double>("loop_rate_hz", 200.0);

    // Modelo del motor, calibracion y limites (config/motorXM430W350T_params.yaml)
    hw_ = omx::declareHwConfig(*this);

    port_name_  = this->get_parameter("port_name").as_string();
    gain_scale_ = this->get_parameter("gain_scale").as_double();
    t_run_      = this->get_parameter("t_run").as_double();
    const int test_num = this->get_parameter("test_num").as_int();
    phi_        = this->get_parameter("phi").as_double();
    const std::string rho_str = this->get_parameter("rho_func").as_string();

    if (rho_str == "sign") {
      rho_func_ = RhoFunc::SIGN;
      rho_str_  = "sign";
    } else {
      rho_func_ = RhoFunc::SAT;
      rho_str_  = "sat";
      if (rho_str != "sat") {
        RCLCPP_WARN(get_logger(),
          "rho_func '%s' desconocida — usando 'sat'", rho_str.c_str());
      }
    }

    ab_alpha_ = get_parameter("ab_alpha").as_double();
    ab_beta_  = get_parameter("ab_beta").as_double();
    loop_rate_hz_ = std::min(std::max(get_parameter("loop_rate_hz").as_double(), 50.0), 400.0);
    Ts_ = 1.0 / loop_rate_hz_;

    RCLCPP_INFO(get_logger(), "puerto=%s  gain=%.2f  t_run(seguimiento)=%.1fs  test=%d",
      port_name_.c_str(), gain_scale_, t_run_, test_num);
    RCLCPP_INFO(get_logger(),
      "SMC articular hw — rho=%s  phi=%.3f  tau_max=%.2f N·m  T_trans=%.1f s",
      rho_str_.c_str(), phi_, hw_.limits.tau_max, T_TRANS);
    RCLCPP_INFO(get_logger(),
      "Lambda=[%.1f %.1f %.1f %.1f]  Kv=[%.1f %.1f %.1f %.1f]  Ks=[%.1f %.1f %.1f %.1f]",
      LAMBDA_Q[0], LAMBDA_Q[1], LAMBDA_Q[2], LAMBDA_Q[3],
      K_V[0],      K_V[1],      K_V[2],      K_V[3],
      K_S[0],      K_S[1],      K_S[2],      K_S[3]);
    RCLCPP_INFO(get_logger(),
      "motor α=[%.1f %.1f %.1f %.1f]  Fv=[%.2f %.2f %.2f %.2f]  ε=%.3f",
      hw_.motor.alpha(0), hw_.motor.alpha(1), hw_.motor.alpha(2), hw_.motor.alpha(3),
      hw_.motor.Fv(0), hw_.motor.Fv(1), hw_.motor.Fv(2), hw_.motor.Fv(3), hw_.motor.epsilon);
    RCLCPP_INFO(get_logger(),
      "hw: tau_max=%.2f N·m  cmd_lim=[%d %d %d %d]  meas_peak=%d  lim_reg=%d",
      hw_.limits.tau_max, hw_.limits.current_cmd_limit[0], hw_.limits.current_cmd_limit[1],
      hw_.limits.current_cmd_limit[2], hw_.limits.current_cmd_limit[3],
      static_cast<int>(hw_.limits.current_measured_peak), static_cast<int>(hw_.limits.current_limit_register));
    RCLCPP_INFO(get_logger(),
      "Filtro α-β: α=%.3f  β=%.4f  |  Fc_scale=%.2f  |  lazo=%.0f Hz",
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
    csv_path_ = std::string(PACKAGE_DATA_DIR) + "/smc/real/act1/hw_smc_joint_"
                + rho_str_ + "_" + std::to_string(test_num) + ".csv";
    omx::openCsv(csv_, csv_path_,
      "t,q1,q2,q3,q4,dq1,dq2,dq3,dq4,"
      "q1_des,q2_des,q3_des,q4_des,dq1_des,dq2_des,dq3_des,dq4_des,"
      "s1,s2,s3,s4,"
      "tau1,tau2,tau3,tau4,"
      "dq1_filt,dq2_filt,dq3_filt,dq4_filt,"
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

  ~HWSMCJointNode()
  {
    if (timer_) timer_->cancel();
    arm_.close();
    if (csv_.is_open()) { csv_.flush(); csv_.close(); }
    RCLCPP_INFO(get_logger(), "Nodo finalizado.");
  }

private:
  // ─────────────────────────────────────────────────────────────────────────
  //  Ley de control SMC articular
  //  gain_scale escala K_V y K_S linealmente (autoridad de alcance y
  //  conmutacion); Lambda define la superficie y no se escala.
  // ─────────────────────────────────────────────────────────────────────────

  Vec4 compute_torque(const Vec4& q, const Vec4& dq, const Reference& ref, Vec4& s_out)
  {
    const Vec4 k_v = K_V * gain_scale_;
    const Vec4 k_s = K_S * gain_scale_;

    // Dinamica nominal M(q) y Phi(q,dq) (infraestructura, no modificar)
    const Eigen::Matrix4d M4   = model_.massMatrix(q);
    const Vec4            nle4 = model_.nonlinearEffects(q, dq);

    // ═══════════════════════════════════════════════════════════════════════
    //  [SECCION 4] Ley SMC articular — COMPLETAR
    //
    //  Disponible:
    //    q, dq                  — posicion y velocidad articular (dq filtrada)
    //    ref.q, ref.dq, ref.ddq — referencia y sus derivadas analiticas
    //    M4 (4x4), nle4 (4x1)   — dinamica nominal M(q) y Phi(q,dq)
    //    LAMBDA_Q, k_v, k_s     — ganancias (k_v, k_s ya escaladas)
    //    rho_vec(s, rho_func_, phi_) — funcion de conmutacion vectorial
    //
    //  Ley (guia SMC; en hardware SIN tau_fric del URDF — la friccion
    //  real se compensa en torque_to_current con el modelo del motor):
    //    e_q  = q - q_d          e_dq = dq - dq_d
    //    s_q  = e_dq + Lambda_q .* e_q          (asignar a s_out)
    //    v_q  = ddq_d - Lambda_q.*e_dq - k_v.*s_q - k_s.*rho(s_q)
    //    tau  = M4 * v_q + nle4
    // ═══════════════════════════════════════════════════════════════════════

    // 4.1 Errores articulares:
    const Vec4 e_q  = Vec4::Zero();   // COMPLETAR
    const Vec4 e_dq = Vec4::Zero();   // COMPLETAR

    // 4.2 Superficie deslizante:
    const Vec4 s_q = Vec4::Zero();    // COMPLETAR

    // 4.3 Funcion de conmutacion: rho = rho_vec(s_q, ...)
    const Vec4 rho = Vec4::Zero();    // COMPLETAR

    // 4.4 Aceleracion articular virtual:
    const Vec4 v_q = Vec4::Zero();    // COMPLETAR

    // 4.5 Torque:
    const Vec4 tau = Vec4::Zero();    // COMPLETAR

    s_out = s_q;

    // Suprimir warnings mientras la seccion esta incompleta (eliminar al completarla):
    (void)M4; (void)nle4; (void)e_q; (void)e_dq; (void)rho; (void)v_q;
    (void)k_v; (void)k_s; (void)ref;
    // ═══════════════════════════════════════════════════════════════════════

    return tau;
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Parada de emergencia centralizada
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
    const auto tp  = std::chrono::high_resolution_clock::now();
    const double t = std::chrono::duration<double>(tp - start_time_).count();

    const double t_shutdown = T_TRANS + t_run_ + RETURN_TIME_S + HOLD_TIME_S;
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

    // 1b. Filtro α-β: estimación conjunta de posición y velocidad
    //   predicción: q_pred = q_hat + Ts·dq_hat,  dq_pred = dq_hat
    //   residuo:    r      = q_meas - q_pred
    //   corrección: q_hat  = q_pred + α·r,        dq_hat = dq_pred + (β/Ts)·r
    if (!ab_initialized_) {
      q_hat_       = q;
      dq_hat_      = dq;
      ab_initialized_ = true;
    } else {
      const Vec4 q_pred  = q_hat_ + Ts_ * dq_hat_;
      const Vec4 dq_pred = dq_hat_;
      const Vec4 r       = q - q_pred;
      q_hat_  = q_pred  + ab_alpha_ * r;
      dq_hat_ = dq_pred + (ab_beta_ / Ts_) * r;
    }

    // 2. Captura posición inicial (primer tick)
    if (!q_initial_captured_) {
      q_initial_ = q;
      q_initial_captured_ = true;
      RCLCPP_INFO(get_logger(), "q_inicial=[%.3f %.3f %.3f %.3f] rad",
        q(0), q(1), q(2), q(3));
    }

    // 3. Referencia: transicion quintica C2 hacia q_d(0), seguimiento, y
    //    retorno quintico final + pausa en reposo (ver RETURN_TIME_S/HOLD_TIME_S)
    const Reference goal0 = desiredTrajectory(0.0);
    const bool in_trans    = (t < T_TRANS);
    const bool in_tracking = !in_trans && (t_run_ <= 0.0 || t < T_TRANS + t_run_);
    Reference ref;
    if (in_trans) {
      ref = omx::quinticTransition(t, T_TRANS, q_initial_, goal0);
    } else if (in_tracking) {
      ref = desiredTrajectory(t - T_TRANS);
    } else if (t < T_TRANS + t_run_ + RETURN_TIME_S) {
      if (!return_logged_) {
        RCLCPP_INFO(get_logger(), "Iniciando retorno quintico a q_inicial (t=%.1fs, dur=%.1fs)",
          t, RETURN_TIME_S);
        return_logged_ = true;
      }
      ref = omx::quinticReturn(t - (T_TRANS + t_run_), RETURN_TIME_S, desiredTrajectory(t_run_), q_initial_);
    } else {
      ref.q = q_initial_;
      ref.dq.setZero();
      ref.ddq.setZero();
    }

    // 4. Verificar límites: referencia y estado medido
    for (int i = 0; i < NUM_JOINTS; ++i) {
      if (ref.q(i) < hw_.limits.joint_lower(i) + 0.02 || ref.q(i) > hw_.limits.joint_upper(i) - 0.02) {
        emergency_stop("Referencia fuera de límites articulares");
        return;
      }
      if (q(i) < hw_.limits.joint_lower(i) || q(i) > hw_.limits.joint_upper(i)) {
        emergency_stop("Articulacion " + std::to_string(i+1) + " fuera de limites: "
                       + std::to_string(q(i)) + " rad");
        return;
      }
    }

    // 5. Ley de control — usa dq_hat_ para reducir chattering por ruido de encoder
    Vec4 s_q;
    const Vec4 tau_unsat = compute_torque(q, dq_hat_, ref, s_q);
    const Vec4 tau = tau_unsat.cwiseMin(hw_.limits.tau_max).cwiseMax(-hw_.limits.tau_max);
    const auto cur_cmd = omx::torqueToCurrent(hw_, tau, dq_hat_, ref.dq);

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

    // 9. CSV (incluye la transicion inicial y el seguimiento; excluye el
    //    retorno final a q_inicial y la pausa en reposo. En hardware las
    //    fallas suelen ocurrir en la transicion; plots_SMC_joint.m modo
    //    'real' descarta t < T_TRANS)
    const bool in_return_or_hold = (t_run_ > 0.0 && t >= T_TRANS + t_run_);
    if (csv_.is_open() && !in_return_or_hold) {
      csv_ << std::fixed << std::setprecision(6) << t
           << ',' << q(0)       << ',' << q(1)       << ',' << q(2)       << ',' << q(3)
           << ',' << dq(0)      << ',' << dq(1)      << ',' << dq(2)      << ',' << dq(3)
           << ',' << ref.q(0)   << ',' << ref.q(1)   << ',' << ref.q(2)   << ',' << ref.q(3)
           << ',' << ref.dq(0)  << ',' << ref.dq(1)  << ',' << ref.dq(2)  << ',' << ref.dq(3)
           << ',' << s_q(0)     << ',' << s_q(1)     << ',' << s_q(2)     << ',' << s_q(3)
           << ',' << tau(0)     << ',' << tau(1)     << ',' << tau(2)     << ',' << tau(3)
           << ',' << dq_hat_(0) << ',' << dq_hat_(1) << ',' << dq_hat_(2) << ',' << dq_hat_(3)
           << ',' << cur_cmd[0] << ',' << cur_cmd[1]  << ',' << cur_cmd[2] << ',' << cur_cmd[3]
           << ',' << cur_meas[0]<< ',' << cur_meas[1] << ',' << cur_meas[2]<< ',' << cur_meas[3]
           << '\n';
    }

    // 10. Log periódico por consola (~1 s)
    if (++log_cnt_ % static_cast<int>(std::lround(loop_rate_hz_)) == 0) {
      if (csv_.is_open()) csv_.flush();
      const char* phase = in_trans    ? "TRANS" :
                          in_tracking ? "TRAY " :
                          (t < T_TRANS + t_run_ + RETURN_TIME_S) ? "RET  " : "HOME ";
      RCLCPP_INFO(get_logger(),
        "[%s] t=%.2fs  |s|=%.4f  |e|=%.4f rad  i=[%d %d %d %d]",
        phase, t, s_q.norm(), (q - ref.q).norm(),
        cur_cmd[0], cur_cmd[1], cur_cmd[2], cur_cmd[3]);
    }

    // 11. Detección de overrun: si el ciclo (bus + control) no cabe en Ts,
    //     el lazo real corre más lento de lo configurado → bajar loop_rate_hz
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
  RhoFunc     rho_func_{RhoFunc::SAT};
  std::string rho_str_;
  double      phi_;

  omx::HwConfig hw_;   // modelo del motor, calibracion y limites (YAML del motor)

  double ab_alpha_, ab_beta_;
  double loop_rate_hz_{200.0};
  double Ts_{0.005};
  Vec4   q_hat_, dq_hat_;
  bool   ab_initialized_{false};

  omx::DynamixelArm arm_{this->get_logger()};

  bool q_initial_captured_;
  Vec4 q_initial_;
  bool return_logged_{false};
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

  const rclcpp::NodeOptions opts = omx::motorParamsNodeOptions(argc, argv, "hw_smc_joint_node");

  try {
    rclcpp::spin(std::make_shared<HWSMCJointNode>(opts));
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("hw_smc_joint_node"), "Excepcion: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
