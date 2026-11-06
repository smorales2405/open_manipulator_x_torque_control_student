// ============================================================================
//  hw_SMC_joint_node_student.cpp
//  Control por Modo Deslizante en espacio articular — OpenMANIPULATOR-X (robot real)
//  [Archivo base para estudiantes — SMC, implementacion]
//
//  Modelo dinamico nominal (Guia SMC, Sec. 6.1):
//    M(q)*ddq + Phi(q,dq) = tau - tau_fric
//    En hardware la friccion REAL se compensa en el modelo torque→corriente
//    identificado (config/control/motorXM430W350T_params.yaml: Fv, Fc en ticks,
//    con Fc alimentado por la velocidad DESEADA dq_d). NO agregar tau_fric del
//    URDF en la ley: seria doble compensacion.
//
//  Parte robusta escalada por diag(M) (solo en hardware):
//    tau = M(q)*v_eq + Phi(q,dq) - diag(M(q))*a_rob
//      v_eq  : control equivalente (aceleracion de referencia)
//      a_rob : alcance robusto, K_V*s + K_S*rho(s) + z
//    Con la M completa el torque robusto de un eje entra a los demas por M_ij;
//    en el robot eso dejaba pegadas a J3 y J4 en las inversiones.
//
//  Termino integral de la ley de alcance (parametro integral):
//    dz/dt = K_I*s,   |z_i| <= Z_MAX_i,   z(0) = 0
//    integral:=false -> z = 0 (SMC clasico)
//    integral:=true  -> z integra en la transicion y el seguimiento, y se congela
//                       en el retorno final y mientras el torque o la corriente
//                       saturan.
//    La actualizacion de z_ y el anti-windup ya estan implementados: en la ley
//    solo se suma z_ al alcance robusto (ver SECCION de la ley de control).
//
//  Funcion de conmutacion (aplicada elemento a elemento):
//    "sat"   ->  rho(s) = sat(s / phi)        phi: capa limite [rad/s]
//
//  Transicion inicial: quintica generalizada de duracion T_TRANS desde la
//  pose medida al arrancar hasta q_d(0), empalmando posicion, velocidad
//  y aceleracion con la trayectoria. El CSV registra la transicion y el
//  seguimiento, sin retorno ni pausa; los scripts plots_SMC_* descartan
//  t < T_TRANS para que las metricas sean solo de seguimiento.
//
//  Fases (con t_run = 0 no hay limite ni retorno):
//    [0, T_TRANS)                     transicion quintica hasta q_d(0)
//    [T_TRANS, T_TRANS+t_run)         seguimiento de la trayectoria
//    [T_TRANS+t_run, +RETURN_TIME_S)  retorno articular al reposo en la pose inicial
//    [..., +HOLD_TIME_S)              pausa en reposo; luego corte de corriente y fin
//  El retorno evita que el brazo caiga por gravedad al recibir torque cero en
//  medio de la trayectoria; usa su propio control, no la ley de la SECCION 4.
//
//  Arranque: el control inicia solo con el robot en la posicion articular
//  inicial de config/control/hw_initial_pose.yaml; fuera de ese rango el nodo
//  muestra donde colocarlo y espera ENTER.
//
//  Lazo de control: 200 Hz de tiempo real (loop_rate_hz). La ley usa la
//  velocidad estimada por el filtro alfa-beta.
//
//  Dynamixel SDK directo sobre /dev/ttyUSB0, sin ros2_control.
//  Publicador : /hw/joint_states  (sensor_msgs/JointState, solo monitoreo)
//
//  Parametros ROS 2 (--ros-args -p nombre:=valor):
//    port_name          [string]  "/dev/ttyUSB0"
//    gain_scale         [double]  1.0    — escala K_V y K_S; K_I con su cuadrado
//    t_run              [double]  20.0   — duracion del seguimiento en segundos;
//                                          total = T_TRANS + t_run +
//                                          RETURN_TIME_S + HOLD_TIME_S
//    test_num           [int]     1      — identificador del CSV generado
//    integral           [bool]    false  — activa el termino integral z
//    ab_alpha           [double]  0.28   — filtro alfa-beta: ganancia de posicion
//    ab_beta            [double]  0.04   — filtro alfa-beta: ganancia de velocidad
//    friction_fc_scale  [double]  0.95   — fraccion de Fc compensada
//    loop_rate_hz       [double]  200.0  — frecuencia del lazo [Hz], en [50, 400]
//  El modelo del motor, la calibracion y los limites se cargan solos desde
//  config/control/motorXM430W350T_params.yaml.
//
//  CSV generado: data/smc/real/act1/hw_smc_joint_sat_<test_num>.csv
//  Columnas: t, q1..q4, dq1..dq4, q1_des..q4_des, dq1_des..dq4_des,
//            s1..s4, tau1..tau4, dq1_filt..dq4_filt, curr_cmd1..4,
//            curr_meas1..4, z1..z4
//  El nombre no distingue integral: usar un test_num distinto en cada ensayo.
//
//  Ejemplos de uso (primer run SIEMPRE con gain_scale:=0.5):
//
//    ros2 run open_manipulator_x_torque_control hw_smc_joint_node_student --ros-args -p gain_scale:=0.5 -p test_num:=1 -p t_run:=25.0
//
//    ros2 run open_manipulator_x_torque_control hw_smc_joint_node_student --ros-args -p integral:=true -p gain_scale:=0.5 -p test_num:=2 -p t_run:=25.0
//
//  ADVERTENCIA: no ejecutar junto a hardware.launch.py ni a otro proceso que
//  use /dev/ttyUSB0 (ros2_control_node, dynamixel_hardware_interface).
//
//  ──────────────────────────────────────────────────────────────────────────
//  SECCIONES A COMPLETAR:
//    [1] Trayectoria articular de referencia  →  desiredTrajectory()
//    [2] Funcion de conmutacion               →  rho_scalar(), PHI
//    [3] Ganancias SMC                        →  LAMBDA_Q, K_V, K_S, K_I
//    [4] Ley SMC articular                    →  compute_torque()
// ============================================================================

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
#include "omx/initial_pose.hpp"
#include "omx/joint_return.hpp"
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

static constexpr double PI = 3.14159265358979323846;
static constexpr int    NUM_JOINTS = 4;
// Calibracion, limites y modelo del motor: omx::declareHwConfig
// (config/control/motorXM430W350T_params.yaml como parametros ROS 2).

// Duracion de la transicion quintica desde la pose medida hasta q_d(0)
// (no modificar)
static constexpr double T_TRANS = 3.0;   // [s]
// Duraciones del retorno final a la pose inicial y de la pausa en reposo antes
// de cortar la corriente (no modificar)
static constexpr double RETURN_TIME_S = 3.0;   // [s]
static constexpr double HOLD_TIME_S   = 0.5;   // [s]

using Vec4 = Eigen::Matrix<double, NUM_JOINTS, 1>;

// ═══════════════════════════════════════════════════════════════════════════
//  [SECCION 3] GANANCIAS SMC — COMPLETAR
//  Ajustar los valores para cada articulacion [joint1, joint2, joint3, joint4]
//
//  Rangos recomendados (lazo de 200 Hz, Ts = 5 ms; con PHI = 0.3):
//    LAMBDA_Q : 15 – 45   [1/s]     (ancho de banda de la superficie)
//    K_V      : 10 – 60             (alcance exponencial; proporcional a s)
//    K_S      : 1  – 8    [rad/s²]  (ganancia de conmutacion)
//    K_I      : 5  – 80   [1/s²]    (integral de s; solo con integral:=true)
//
// ═══════════════════════════════════════════════════════════════════════════
static const Vec4 LAMBDA_Q = (Vec4() << 0.0, 0.0, 0.0, 0.0).finished();  // COMPLETAR
static const Vec4 K_V      = (Vec4() << 0.0, 0.0, 0.0, 0.0).finished();  // COMPLETAR
static const Vec4 K_S      = (Vec4() << 0.0, 0.0, 0.0, 0.0).finished();  // COMPLETAR
static const Vec4 K_I      = (Vec4() << 0.0, 0.0, 0.0, 0.0).finished();  // COMPLETAR
// ═══════════════════════════════════════════════════════════════════════════

// Tope anti-windup del termino integral |z_i| <= Z_MAX_i [rad/s²]
static const Vec4 Z_MAX = (Vec4() << 30.0, 80.0, 120.0, 400.0).finished();

// ── Estructura de referencia articular ───────────────────────────────────────
using omx::Reference;   // {q, dq, ddq}

// ─────────────────────────────────────────────────────────────────────────────
//  [SECCION 1] Trayectoria articular de referencia — COMPLETAR
//
//  Definir la trayectoria para cada articulacion de la forma:
//    qi_d(t)   = Ai + Bi * sin(w*t)
//    dqi_d(t)  = Bi * w * cos(w*t)
//    ddqi_d(t) = -Bi * w^2 * sin(w*t)
//
//  Usar w = 1.0 rad/s.
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
//  [SECCION 2] Funcion de conmutacion — COMPLETAR
//
//  Implementar la saturacion (funcion unica en este nodo):
//    sat  : rho(s) = clamp(s / phi, -1, 1)        (phi: capa limite)
//
//  Definir tambien PHI, la capa limite que el nodo pasa como phi a sat:
//    PHI : 0.2 – 0.5 [rad/s], comun a las 4 articulaciones.
// ─────────────────────────────────────────────────────────────────────────────
static const double PHI = 0.0;   // COMPLETAR [rad/s]

static double rho_scalar(double s, double phi)
{
  (void)s; (void)phi;

  return 0.0;  // COMPLETAR: sat(s/phi)
}

// Aplica rho_scalar elemento a elemento sobre el vector s (no modificar)
[[maybe_unused]] static Vec4 rho_vec(const Vec4 & s, double phi)
{
  Vec4 r;
  for (int i = 0; i < NUM_JOINTS; ++i) {
    r[i] = rho_scalar(s[i], phi);
  }
  return r;
}

// ── Nodo principal ────────────────────────────────────────────────────────────

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
    this->declare_parameter<bool>       ("integral",       false);

    this->declare_parameter<double>("ab_alpha", 0.28);
    this->declare_parameter<double>("ab_beta",  0.04);
    this->declare_parameter<double>("loop_rate_hz", 200.0);

    // Modelo del motor, calibracion y limites (config/control/motorXM430W350T_params.yaml)
    hw_ = omx::declareHwConfig(*this);

    port_name_  = this->get_parameter("port_name").as_string();
    gain_scale_ = this->get_parameter("gain_scale").as_double();
    t_run_      = this->get_parameter("t_run").as_double();
    const int test_num = this->get_parameter("test_num").as_int();
    integral_   = this->get_parameter("integral").as_bool();

    ab_alpha_ = get_parameter("ab_alpha").as_double();
    ab_beta_  = get_parameter("ab_beta").as_double();
    loop_rate_hz_ = std::min(std::max(get_parameter("loop_rate_hz").as_double(), 50.0), 400.0);
    Ts_ = 1.0 / loop_rate_hz_;

    // PHI se completa en la SECCION 2: sat(s/PHI) necesita PHI > 0. Se
    // verifica antes de abrir el puerto y habilitar torque.
    if (!(PHI > 0.0)) {
      RCLCPP_FATAL(get_logger(),
        "PHI = %.3f: completar PHI > 0 en la SECCION 2 (capa limite de sat)", PHI);
      throw std::runtime_error("PHI sin completar");
    }

    RCLCPP_INFO(get_logger(), "puerto=%s  gain=%.2f  t_run(seguimiento)=%.1fs  test=%d",
      port_name_.c_str(), gain_scale_, t_run_, test_num);
    RCLCPP_INFO(get_logger(),
      "SMC articular hw — rho=sat  PHI=%.3f  integral=%s  tau_max=%.2f N·m  T_trans=%.1f s",
      PHI, integral_ ? "true" : "false", hw_.limits.tau_max, T_TRANS);
    RCLCPP_INFO(get_logger(),
      "Lambda=[%.1f %.1f %.1f %.1f]  Kv=[%.1f %.1f %.1f %.1f]  Ks=[%.1f %.1f %.1f %.1f]",
      LAMBDA_Q[0], LAMBDA_Q[1], LAMBDA_Q[2], LAMBDA_Q[3],
      K_V[0],      K_V[1],      K_V[2],      K_V[3],
      K_S[0],      K_S[1],      K_S[2],      K_S[3]);
    if (integral_) {
      RCLCPP_INFO(get_logger(),
        "Ki=[%.1f %.1f %.1f %.1f]  z_max=[%.1f %.1f %.1f %.1f]",
        K_I[0],   K_I[1],   K_I[2],   K_I[3],
        Z_MAX[0], Z_MAX[1], Z_MAX[2], Z_MAX[3]);
    }
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

    // ── Publisher de monitoreo ────────────────────────────────────────────────
    js_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/hw/joint_states", 10);

    // ── SDK ──────────────────────────────────────────────────────────────────
    if (!arm_.open(port_name_, hw_.calib, hw_.limits.current_limit_register)) {
      RCLCPP_FATAL(get_logger(), "Fallo hardware init. Abortando.");
      throw std::runtime_error("Hardware init failed");
    }

    // ── Posicion articular inicial (config/control/hw_initial_pose.yaml) ─────
    // Espera a que el robot este en su posicion inicial. El CSV se abre despues:
    // salir aqui con Ctrl+C no borra un CSV anterior con el mismo numero de prueba.
    if (!omx::waitForInitialPose(*this, arm_)) {
      return;
    }

    // ── CSV ──────────────────────────────────────────────────────────────────
    csv_path_ = std::string(PACKAGE_DATA_DIR) + "/smc/real/act1/hw_smc_joint_sat_"
                + std::to_string(test_num) + ".csv";
    omx::openCsv(csv_, csv_path_,
      "t,q1,q2,q3,q4,dq1,dq2,dq3,dq4,"
      "q1_des,q2_des,q3_des,q4_des,dq1_des,dq2_des,dq3_des,dq4_des,"
      "s1,s2,s3,s4,"
      "tau1,tau2,tau3,tau4,"
      "dq1_filt,dq2_filt,dq3_filt,dq4_filt,"
      "curr_cmd1,curr_cmd2,curr_cmd3,curr_cmd4,"
      "curr_meas1,curr_meas2,curr_meas3,curr_meas4,"
      "z1,z2,z3,z4",
      get_logger());

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
  // Termino integral de la ley de alcance (infraestructura, no modificar):
  // dz/dt = K_I*s, con tope |z_i| <= Z_MAX_i. Vale cero con integral:=false y
  // se congela si el tick anterior saturo torque o corriente. gain_scale escala
// K_I con su cuadrado.
  void update_integral(const Vec4& s, bool integrate)
  {
    if (integrate && !tau_saturated_prev_) {
      z_ += Ts_ * (K_I * gain_scale_ * gain_scale_).cwiseProduct(s);
      z_ = z_.cwiseMin(Z_MAX).cwiseMax(-Z_MAX);
    }
  }

  // ── Ley de control (se llama en cada tick con dq filtrada) ─────────────────
  Vec4 compute_torque(const Vec4& q, const Vec4& dq, const Reference& ref,
                      bool integrate, Vec4& s_out)
  {
    // gain_scale escala K_V y K_S (Lambda define la superficie y no se escala)
    const Vec4 k_v = K_V * gain_scale_;
    const Vec4 k_s = K_S * gain_scale_;

    // ── Dinamica nominal via Pinocchio ─────────────────────────────────────
    const Eigen::Matrix4d M4   = model_.massMatrix(q);
    const Vec4            nle4 = model_.nonlinearEffects(q, dq);

    // ─────────────────────────────────────────────────────────────────────────
    //  [SECCION 4] Ley SMC articular — COMPLETAR
    //  Disponible: M4 (4x4), nle4 (4x1), q, dq (filtrada), ref.q, ref.dq,
    //              ref.ddq, LAMBDA_Q, k_v, k_s (ya escaladas),
    //              z_ (4x1, termino integral), PHI
    //  diag(M4) = M4.diagonal(); producto elemento a elemento: cwiseProduct
    // ─────────────────────────────────────────────────────────────────────────

    //  4.1 Errores articulares:
    const Vec4 e_q  = Vec4::Zero();
    const Vec4 e_dq = Vec4::Zero();

    // 4.2 Superficie deslizante:
    // COMPLETAR
    const Vec4 s_q = Vec4::Zero();

    // 4.3 Funcion de conmutacion: rho = rho_vec(s_q, ...)
    // COMPLETAR
    const Vec4 rho = Vec4::Zero();

    // Termino integral z_ (infraestructura, no modificar):
    update_integral(s_q, integrate);

    // 4.4 Control equivalente:
    // COMPLETAR
    const Vec4 v_eq = Vec4::Zero();

    // 4.5 Alcance robusto:
    // COMPLETAR
    const Vec4 a_rob = Vec4::Zero();

    // 4.6 Torque:
    const Vec4 tau = Vec4::Zero();

    s_out = s_q;

    // Suprimir warnings mientras la seccion esta incompleta (eliminar al completarla):
    (void)M4; (void)nle4; (void)e_q; (void)e_dq; (void)rho; (void)v_eq; (void)a_rob;
    (void)k_v; (void)k_s; (void)ref;
    // ─────────────────────────────────────────────────────────────────────────

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
      ret_.setHome(q);   // pose articular a la que vuelve el retorno final
      q_initial_captured_ = true;
      RCLCPP_INFO(get_logger(), "q_inicial=[%.3f %.3f %.3f %.3f] rad",
        q(0), q(1), q(2), q(3));
    }

    // 3. Referencia: transicion quintica C2 hacia q_d(0), seguimiento, y
    //    retorno articular final + pausa en reposo (ver RETURN_TIME_S/HOLD_TIME_S)
    const Reference goal0 = desiredTrajectory(0.0);
    const bool in_trans    = (t < T_TRANS);
    const bool in_tracking = !in_trans && (t_run_ <= 0.0 || t < T_TRANS + t_run_);
    const bool returning   = !in_trans && !in_tracking;
    Reference ref;
    if (in_trans) {
      ref = omx::quinticTransition(t, T_TRANS, q_initial_, goal0);
    } else if (in_tracking) {
      ref = desiredTrajectory(t - T_TRANS);
    } else {
      // Quintica desde la pose medida en t_run hasta q_inicial en reposo
      if (!ret_.started()) {
        ret_.start(q);
        RCLCPP_INFO(get_logger(), "Iniciando retorno articular a q_inicial (t=%.1fs, dur=%.1fs)",
          t, RETURN_TIME_S);
      }
      ref = ret_.reference(t - (T_TRANS + t_run_));
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

    // 5. Ley de control — usa dq_hat_ para reducir chattering por ruido de encoder.
    //    En el retorno final el par lo da el retorno articular a q_inicial (la
    //    ley SMC se evalua igual, pero su par se descarta).
    Vec4 s_q;
    const bool integrate = integral_ && (in_trans || in_tracking);
    const Vec4 tau_smc   = compute_torque(q, dq_hat_, ref, integrate, s_q);
    const Vec4 tau_unsat = returning ? Vec4(ret_.torque(model_, q, dq_hat_, ref)) : tau_smc;
    const Vec4 tau = tau_unsat.cwiseMin(hw_.limits.tau_max).cwiseMax(-hw_.limits.tau_max);
    std::array<bool, NUM_JOINTS> clipped{};
    const auto cur_cmd = omx::torqueToCurrent(hw_, tau, dq_hat_, ref.dq, &clipped);
    tau_saturated_prev_ = (tau_unsat.cwiseAbs().maxCoeff() >= hw_.limits.tau_max)
                        || clipped[0] || clipped[1] || clipped[2] || clipped[3];

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
           << ',' << z_(0)      << ',' << z_(1)       << ',' << z_(2)      << ',' << z_(3)
           << '\n';
    }

    // 10. Log periódico por consola (~1 s)
    if (++log_cnt_ % static_cast<int>(std::lround(loop_rate_hz_)) == 0) {
      if (csv_.is_open()) csv_.flush();
      const char* phase = in_trans    ? "TRANS" :
                          in_tracking ? "TRAY " :
                          (t < T_TRANS + t_run_ + RETURN_TIME_S) ? "RET  " : "HOME ";
      RCLCPP_INFO(get_logger(),
        "[%s] t=%.2fs  |s|=%.4f  |e|=%.4f rad  |z|=%.2f  i=[%d %d %d %d]",
        phase, t, s_q.norm(), (q - ref.q).norm(), z_.norm(),
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

  bool integral_{false};
  Vec4 z_{Vec4::Zero()};              // termino integral [rad/s²]
  bool tau_saturated_prev_{false};    // anti-windup

  omx::HwConfig hw_;   // modelo del motor, calibracion y limites (YAML del motor)

  double ab_alpha_, ab_beta_;
  double loop_rate_hz_{200.0};
  double Ts_{0.005};
  Vec4   q_hat_, dq_hat_;
  bool   ab_initialized_{false};

  omx::DynamixelArm arm_{this->get_logger()};

  bool q_initial_captured_;
  Vec4 q_initial_;
  omx::JointReturn ret_{omx::JointReturn::forHardware(RETURN_TIME_S)};   // retorno articular final a q_inicial
  std::chrono::high_resolution_clock::time_point start_time_;
  int log_cnt_{0};
  int overrun_cnt_{0};

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::ofstream csv_;
  std::string   csv_path_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  const rclcpp::NodeOptions opts = omx::motorParamsNodeOptions(argc, argv, "hw_smc_joint_node");

  try {
    auto node = std::make_shared<HWSMCJointNode>(opts);
    if (rclcpp::ok()) {   // false: Ctrl+C durante la espera de la posicion inicial
      rclcpp::spin(node);
    }
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("hw_smc_joint_node"), "Excepcion: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
