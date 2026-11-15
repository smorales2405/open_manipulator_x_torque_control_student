// ============================================================================
//  hw_MRAC_joint_12p_node_student.cpp
//  Control Adaptativo (MRAC / Slotine-Li) articular — 12 PARAMETROS
//  OpenMANIPULATOR-X (robot real)
//  [Archivo base para estudiantes — MRAC, implementacion]
//
//  Modelo dinamico nominal (Guia SMC, Sec. 6.1):
//    M(q)*ddq + Phi(q,dq) = tau - tau_fric
//    En hardware la friccion REAL se compensa en el modelo torque→corriente
//    identificado (config/control/motorXM430W350T_params.yaml: Fv, Fc en ticks,
//    con Fc alimentado por la velocidad DESEADA dq_d). NO agregar la friccion
//    del URDF en la ley: seria doble compensacion.
//
//  Parametros adaptados (12):
//    a_hat = [ alpha1..alpha3 , dm, dmcx, dmcy, dmcz , Fv1_res , Fc1..Fc4_res ]^T
//      alpha_k   : escala de inercia del cuerpo movido por jointk, nominal 1.0
//      theta_load: exceso de masa [kg] y de primer momento [kg·m] del ultimo
//                  cuerpo (link5 + lo que sujete el gripper), nominal 0.
//                  Cerrar el gripper sobre la carga ANTES de correr el nodo.
//      Fv1_res, Fc_res: friccion RESIDUAL (desviacion respecto a la
//                  identificada) en dominio de torque, nominal 0
//    El regresor Y (4x12), el prior a_prior, las tasas Gamma, la fuga sigma,
//    las cotas, la zona muerta y la compuerta de la adaptacion ya estan
//    implementados.
//
//  Realimentacion escalada por diag(M) (solo en hardware):
//    tau = M(q)*ddq_r + Phi(q,dq) + Y*(a_hat - a_prior) - diag(M(q))*a_rob
//      a_rob : alcance, K_V*s + K_S*rho(s)
//    Con la M completa la realimentacion de un eje entra a los demas por M_ij;
//    en el robot eso dejaba pegadas a J3 y J4 en las inversiones.
//
//  Ley de adaptacion: a_hat_dot = -Gamma .* ( Y^T*s_dz + sigma .* (a_hat - a_prior) ),
//    con s_dz = s sin la zona muerta |s| <= adapt_deadzone. La integracion,
//    la proyeccion a las cotas y el congelamiento (retorno final, saturacion
//    de torque o corriente, o error mayor a ADAPT_EMAX) ya estan implementados.
//    Con adaptive:=false, a_hat queda en a_prior.
//
//  Funcion de conmutacion (aplicada elemento a elemento):
//    "sat"   ->  rho(s) = sat(s / phi)        phi: capa limite [rad/s]
//
//  Transicion inicial: quintica de duracion T_TRANS desde la pose medida al
//  arrancar hasta q_d(0), empalmando posicion, velocidad y aceleracion. El
//  CSV registra la transicion y el seguimiento, sin retorno ni pausa; los
//  scripts descartan t < T_TRANS para que las metricas sean solo de
//  seguimiento.
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
//    adaptive           [bool]    true   — false: a_hat fijo en a_prior
//    gain_scale         [double]  1.0    — escala K_V y K_S
//    t_run              [double]  20.0   — duracion del seguimiento en segundos;
//                                          total = T_TRANS + t_run +
//                                          RETURN_TIME_S + HOLD_TIME_S
//    test_num           [int]     1      — identificador del CSV generado
//    adapt_deadzone     [double]  0.1    — zona muerta de |s| para la adaptacion [rad/s]
//    gamma_scale        [double]  1.0    — escala todas las tasas Gamma
//    ab_alpha           [double]  0.28   — filtro alfa-beta: ganancia de posicion
//    ab_beta            [double]  0.04   — filtro alfa-beta: ganancia de velocidad
//    friction_fc_scale  [double]  0.95   — fraccion de Fc compensada
//    loop_rate_hz       [double]  200.0  — frecuencia del lazo [Hz], en [50, 400]
//  El modelo del motor, la calibracion y los limites se cargan solos desde
//  config/control/motorXM430W350T_params.yaml.
//
//  CSV generado: data/mrac/real/mrac12p/hw_mrac_joint_12p_<modo>_<test_num>.csv
//    <modo> = "adaptive" | "fixed"
//  Columnas: t, q1..q4, dq1..dq4, q1_des..q4_des, dq1_des..dq4_des,
//            s1..s4, tau1..tau4, dq1_filt..dq4_filt, curr_cmd1..4,
//            curr_meas1..4, a1_hat..a3_hat, dm_hat, dmcx_hat, dmcy_hat,
//            dmcz_hat, fv1_hat, fc1_hat..fc4_hat
//
//  Ejemplos de uso (primer run SIEMPRE con gain_scale:=0.5):
//
//    ros2 run open_manipulator_x_torque_control hw_mrac_joint_12p_node_student --ros-args -p adaptive:=false -p gain_scale:=0.5 -p test_num:=1 -p t_run:=25.0
//
//    ros2 run open_manipulator_x_torque_control hw_mrac_joint_12p_node_student --ros-args -p gain_scale:=0.5 -p test_num:=2 -p t_run:=25.0
//
//  ADVERTENCIA: no ejecutar junto a hardware.launch.py ni a otro proceso que
//  use /dev/ttyUSB0 (ros2_control_node, dynamixel_hardware_interface).
//
//  ──────────────────────────────────────────────────────────────────────────
//  SECCIONES A COMPLETAR:
//    [1] Trayectoria articular de referencia  →  desiredTrajectory()
//    [2] Funcion de conmutacion               →  rho_scalar(), PHI
//    [3] Ganancias MRAC                       →  LAMBDA_Q, K_V, K_S
//    [4] Ley de control y adaptacion          →  compute_torque(), update_parameters()
// ============================================================================

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

// ============================================================
// Constantes (servos Dynamixel y unidades: omx/dynamixel_io.hpp)
// ============================================================

static constexpr double PI = 3.14159265358979323846;
static constexpr int    NUM_JOINTS = 4;
static constexpr int    NP         = 12;   // [alpha1..3, theta_load(4), Fv1, Fc1..4]

// Calibracion, limites y modelo del motor: omx::declareHwConfig
// (config/control/motorXM430W350T_params.yaml como parametros ROS 2).

// Duracion de la transicion quintica desde la pose medida hasta q_d(0)
static constexpr double T_TRANS = 3.0;   // [s]
static constexpr double RETURN_TIME_S = 3.0;   // duracion del retorno articular a q_inicial
static constexpr double HOLD_TIME_S   = 0.5;   // pausa en reposo antes de cortar corriente

// Recorte de errores en la ley (anti-rele). E_CLAMP = 0.5 rad: con 0.15 el
// techo de correccion estatica quedaba bajo el breakaway real y el brazo se
// estacionaba por stiction lejos de la referencia (MRAC tests 6-7).
static constexpr double E_CLAMP  = 0.5;   // [rad]   recorte de e_q
static constexpr double DE_CLAMP = 1.5;   // [rad/s] recorte de e_dq

// Compuerta de adaptacion por calidad de tracking: adaptar SOLO cerca de la
// referencia (MRAC test 8: con el brazo estacionado a ~1 rad los parametros
// paseaban de cota a cota).
static constexpr double ADAPT_EMAX = 0.25;  // [rad] max|e_q| para adaptar

using Vec4  = Eigen::Matrix<double, NUM_JOINTS, 1>;
using Vec12 = Eigen::Matrix<double, NP, 1>;
using Vec10 = Eigen::Matrix<double, 10, 1>;
using MatY  = Eigen::Matrix<double, NUM_JOINTS, NP>;

// Indices de bloque dentro de a_hat:
static constexpr int IDX_ALPHA = 0;   // [0..2]  alpha1..alpha3
static constexpr int IDX_LOAD  = 3;   // [3..6]  dm, dmcx, dmcy, dmcz
static constexpr int IDX_FV1   = 7;   // [7]     Fv1 residual
static constexpr int IDX_FC    = 8;   // [8..11] Fc1..Fc4 residual

// ═══════════════════════════════════════════════════════════════════════════
//  [SECCION 3] GANANCIAS MRAC — COMPLETAR
//  Ajustar los valores para cada articulacion [joint1, joint2, joint3, joint4]
//
//  Rangos recomendados (lazo de 200 Hz, Ts = 5 ms; con PHI = 0.3):
//    LAMBDA_Q : 15 – 45   [1/s]     (ancho de banda de la superficie)
//    K_V      : 10 – 60             (alcance exponencial; proporcional a s)
//    K_S      : 1  – 8    [rad/s²]  (ganancia de conmutacion)
//
// ═══════════════════════════════════════════════════════════════════════════
static const Vec4 LAMBDA_Q = (Vec4() << 0.0, 0.0, 0.0, 0.0).finished();  // COMPLETAR
static const Vec4 K_V      = (Vec4() << 0.0, 0.0, 0.0, 0.0).finished();  // COMPLETAR
static const Vec4 K_S      = (Vec4() << 0.0, 0.0, 0.0, 0.0).finished();  // COMPLETAR
// ═══════════════════════════════════════════════════════════════════════════

// Tasas de adaptacion, fuga sigma hacia el prior [1/s] y cotas de los
// parametros, por bloque [alpha1..3, dm, dmcx, dmcy, dmcz, Fv1, Fc1..4]
// (no modificar). En hardware las tasas son ~4x mas frias que en Gazebo:
// la adaptacion promedia sobre el ciclo y estima la componente lenta.
static const Vec12 GAMMA =
  (Vec12() << 0.25, 0.25, 0.25,   0.05, 0.0025, 0.0025, 0.0025,   0.5,   0.5, 0.5, 0.5, 0.5).finished();

static const Vec12 SIGMA_LEAK =
  (Vec12() << 0.1, 0.1, 0.1,   0.02, 0.02, 0.02, 0.02,   0.1,   0.1, 0.1, 0.1, 0.1).finished();

static const Vec12 A_MIN =
  (Vec12() << 0.80, 0.80, 0.80,   -0.05, -0.012, -0.010, -0.005,   -0.02,  -0.04, -0.04, -0.04, -0.04).finished();
static const Vec12 A_MAX =
  (Vec12() << 1.25, 1.25, 1.25,    0.20,  0.035,  0.010,  0.020,    0.05,   0.08,  0.08,  0.08,  0.08).finished();

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
//  Implementar la saturacion:
//    sat  : rho(s) = clamp(s / phi, -1, 1)        (phi: capa limite)
//
//  Definir tambien PHI, la capa limite que el nodo pasa como phi a sat:
//    PHI : 0.2 – 0.5 [rad/s], comun a las 4 articulaciones.
// ─────────────────────────────────────────────────────────────────────────────
static const double PHI = 0.0;   // COMPLETAR [rad/s]

static double rho_scalar(double s, double phi)
{
  (void)s; (void)phi;

  return 0.0;  // COMPLETAR: sat(s/phi) = clamp(s/phi, -1, 1)
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

// ============================================================
// Nodo ROS 2
// ============================================================

class HWMRACJoint12pNode : public rclcpp::Node
{
public:
  explicit HWMRACJoint12pNode(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions())
  : Node("hw_mrac_joint_12p_node", opts), q_initial_captured_(false)
  {
    // ── Parámetros ──────────────────────────────────────────────────────────
    this->declare_parameter<std::string>("port_name",      "/dev/ttyUSB0");
    this->declare_parameter<bool>       ("adaptive",       true);
    this->declare_parameter<double>     ("gain_scale",     1.0);
    this->declare_parameter<double>     ("t_run",          20.0);
    this->declare_parameter<int>        ("test_num",       1);
    this->declare_parameter<double>     ("adapt_deadzone", 0.1);
    this->declare_parameter<double>     ("gamma_scale",    1.0);

    this->declare_parameter<double>("ab_alpha", 0.28);
    this->declare_parameter<double>("ab_beta",  0.04);
    this->declare_parameter<double>("loop_rate_hz", 200.0);

    // Modelo del motor, calibracion y limites (config/control/motorXM430W350T_params.yaml)
    hw_ = omx::declareHwConfig(*this);

    port_name_  = this->get_parameter("port_name").as_string();
    adaptive_   = this->get_parameter("adaptive").as_bool();
    gain_scale_ = this->get_parameter("gain_scale").as_double();
    t_run_      = this->get_parameter("t_run").as_double();
    const int test_num = this->get_parameter("test_num").as_int();
    adapt_dz_   = this->get_parameter("adapt_deadzone").as_double();
    gamma_      = GAMMA * std::max(this->get_parameter("gamma_scale").as_double(), 0.0);
    mode_str_   = adaptive_ ? "adaptive" : "fixed";

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

    // Prior: alphas en 1, carga y friccion residual en 0 (no modificar).
    a_prior_.setZero();
    a_prior_.segment<3>(IDX_ALPHA).setOnes();
    a_hat_ = a_prior_;

    RCLCPP_INFO(get_logger(), "puerto=%s  modo=%s  gain=%.2f  t_run(seguimiento)=%.1fs  test=%d",
      port_name_.c_str(), mode_str_.c_str(), gain_scale_, t_run_, test_num);
    RCLCPP_INFO(get_logger(),
      "MRAC 12p hw — PHI=%.3f  tau_max=%.2f N·m  T_trans=%.1f s  deadzone(s)=%.2f  "
      "clamp e=%.2f rad / edq=%.1f rad/s",
      PHI, hw_.limits.tau_max, T_TRANS, adapt_dz_, E_CLAMP, DE_CLAMP);
    RCLCPP_INFO(get_logger(),
      "Lambda=[%.1f %.1f %.1f %.1f]  Kv=[%.1f %.1f %.1f %.1f]  Ks=[%.1f %.1f %.1f %.1f]",
      LAMBDA_Q[0], LAMBDA_Q[1], LAMBDA_Q[2], LAMBDA_Q[3],
      K_V[0],      K_V[1],      K_V[2],      K_V[3],
      K_S[0],      K_S[1],      K_S[2],      K_S[3]);
    RCLCPP_INFO(get_logger(),
      "Gamma: alpha=%.3f  load=[%.3f %.4f]  Fv1=%.2f  Fc=%.2f  |  sigma load=%.2f resto=%.2f",
      gamma_[0], gamma_[3], gamma_[4], gamma_[7], gamma_[8], SIGMA_LEAK[3], SIGMA_LEAK[0]);
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
    // Parametros dinamicos nominales del cuerpo movido por cada joint (para
    // contraer las columnas alpha del regresor) y bloque de link5 (theta_load)
    for (int i = 0; i < NUM_JOINTS; ++i) {
      const std::string jname = "joint" + std::to_string(i + 1);
      jid_[i]    = model_.jointId(jname);
      pi_nom_[i] = model_.bodyDynamicParameters(jid_[i]);
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
    csv_path_ = std::string(PACKAGE_DATA_DIR) + "/mrac/real/mrac12p/hw_mrac_joint_12p_"
                + mode_str_ + "_" + std::to_string(test_num) + ".csv";
    omx::openCsv(csv_, csv_path_,
      "t,q1,q2,q3,q4,dq1,dq2,dq3,dq4,"
      "q1_des,q2_des,q3_des,q4_des,dq1_des,dq2_des,dq3_des,dq4_des,"
      "s1,s2,s3,s4,"
      "tau1,tau2,tau3,tau4,"
      "dq1_filt,dq2_filt,dq3_filt,dq4_filt,"
      "curr_cmd1,curr_cmd2,curr_cmd3,curr_cmd4,"
      "curr_meas1,curr_meas2,curr_meas3,curr_meas4,"
      "a1_hat,a2_hat,a3_hat,"
      "dm_hat,dmcx_hat,dmcy_hat,dmcz_hat,"
      "fv1_hat,fc1_hat,fc2_hat,fc3_hat,fc4_hat",
      get_logger());

    // ── Timer de control (loop_rate_hz) ──────────────────────────────────────
    start_time_ = std::chrono::high_resolution_clock::now();
    const auto period = std::chrono::microseconds(
      static_cast<int64_t>(std::lround(1e6 / loop_rate_hz_)));
    timer_ = this->create_wall_timer(period, [this]() { tick(); });
    RCLCPP_INFO(get_logger(), "Control activo a %.0f Hz (Ts=%.1f ms). Ctrl+C para detener.",
      loop_rate_hz_, 1e3 * Ts_);
  }

  ~HWMRACJoint12pNode()
  {
    if (timer_) timer_->cancel();
    arm_.close();
    if (csv_.is_open()) { csv_.flush(); csv_.close(); }
    RCLCPP_INFO(get_logger(), "Nodo finalizado.");
  }

private:
  // ─────────────────────────────────────────────────────────────────────────
  //  Regresor 4x12: columnas alpha (bloques de 10 parametros de link2..link4
  //  contraidos con sus valores nominales), theta_load (columnas [m, m*c] del
  //  bloque de link5), Fv1 (dq1 filtrada) y Fc (tanh(dq_d/eps) por junta).
  // ─────────────────────────────────────────────────────────────────────────

  MatY regressor(const Vec4& q, const Vec4& dq, const Vec4& dqr, const Vec4& ddqr,
                 const Vec4& tanh_dqd)
  {
    const Eigen::MatrixXd Y_SL = model_.slotineLiRegressor(q, dq, dqr, ddqr);
    MatY Y = MatY::Zero();
    for (int i = 0; i < 3; ++i) {
      const int col0 = 10 * (static_cast<int>(jid_[i]) - 1);
      Y.col(IDX_ALPHA + i) = (Y_SL.middleCols(col0, 10) * pi_nom_[i]).head<NUM_JOINTS>();
    }
    const int col0_load = 10 * (static_cast<int>(jid_[3]) - 1);
    Y.block<NUM_JOINTS, 4>(0, IDX_LOAD) = Y_SL.block(0, col0_load, NUM_JOINTS, 4);
    Y(0, IDX_FV1) = dq(0);
    for (int i = 0; i < NUM_JOINTS; ++i) {
      Y(i, IDX_FC + i) = tanh_dqd(i);
    }
    return Y;
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Ley de control MRAC articular (devuelve tau sin saturar; s_out y Y_out
  //  quedan para la adaptacion y el CSV)
  // ─────────────────────────────────────────────────────────────────────────

  Vec4 compute_torque(const Vec4& q, const Vec4& dq, const Reference& ref,
                      Vec4& s_out, MatY& Y_out)
  {
    // gain_scale escala K_V y K_S (Lambda define la superficie y no se escala)
    const Vec4 k_v = K_V * gain_scale_;
    const Vec4 k_s = K_S * gain_scale_;

    // Errores recortados (anti-rele; infraestructura, no modificar): cerca de
    // la referencia son los errores e_q = q - q_d y e_dq = dq - dq_d
    const Vec4 e_c   = (q  - ref.q ).cwiseMax(-E_CLAMP).cwiseMin(E_CLAMP);
    const Vec4 edq_c = (dq - ref.dq).cwiseMax(-DE_CLAMP).cwiseMin(DE_CLAMP);

    // ─────────────────────────────────────────────────────────────────────────
    //  [SECCION 4] Ley de control — COMPLETAR
    //  Disponible: e_c, edq_c (errores), ref.q, ref.dq, ref.ddq, q, dq
    //              (filtrada), LAMBDA_Q, k_v, k_s (ya escaladas), PHI,
    //              a_hat_, a_prior_ (12x1)
    //  diag(M) = M.diagonal(); producto elemento a elemento: cwiseProduct
    // ─────────────────────────────────────────────────────────────────────────

    // 4.1 Superficie deslizante y referencias de Slotine-Li (con e_c, edq_c):
    // COMPLETAR
    const Vec4 s_q  = Vec4::Zero();
    const Vec4 dqr  = Vec4::Zero();
    const Vec4 ddqr = Vec4::Zero();

    // Dinamica nominal y regresor (infraestructura, no modificar):
    //   M (4x4), tau_nom = M*ddq_r + Phi(q,dq) (RNEA), Y (4x12)
    const Vec4 tanh_dqd = ((ref.dq / hw_.motor.epsilon).array().tanh()).matrix();
    const Eigen::Matrix4d M = model_.massMatrix(q);
    const Vec4 tau_nom = model_.inverseDynamics(q, dq, ddqr);
    Y_out = regressor(q, dq, dqr, ddqr, tanh_dqd);

    // 4.2 Funcion de conmutacion: rho = rho_vec(s_q, ...)
    // COMPLETAR
    const Vec4 rho = Vec4::Zero();

    // 4.3 Torque:
    const Vec4 tau = Vec4::Zero();

    s_out = s_q;

    // Suprimir warnings mientras la seccion esta incompleta (eliminar al completarla):
    (void)e_c; (void)edq_c; (void)M; (void)tau_nom; (void)rho; (void)k_v; (void)k_s;
    // ─────────────────────────────────────────────────────────────────────────

    return tau;
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Ley de adaptacion (se llama solo cuando la adaptacion esta habilitada)
  // ─────────────────────────────────────────────────────────────────────────

  void update_parameters(const MatY& Y, const Vec4& s)
  {
    // Zona muerta (infraestructura, no modificar): s_dz = s sin |s| <= adapt_deadzone
    Vec4 s_dz;
    for (int i = 0; i < NUM_JOINTS; ++i) {
      const double ex = std::abs(s[i]) - adapt_dz_;
      s_dz[i] = (ex > 0.0) ? (s[i] > 0.0 ? ex : -ex) : 0.0;
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  [SECCION 4, continuacion] Ley de adaptacion — COMPLETAR
    //  Disponible: Y (4x12), s_dz (4x1), gamma_ (Gamma ya escalada),
    //              SIGMA_LEAK, a_hat_, a_prior_ (12x1)
    // ─────────────────────────────────────────────────────────────────────────

    // 4.4 Derivada de a_hat (12x1):
    // COMPLETAR
    const Vec12 a_dot = Vec12::Zero();

    // Suprimir warnings mientras la seccion esta incompleta (eliminar al completarla):
    (void)Y; (void)s_dz;
    // ─────────────────────────────────────────────────────────────────────────

    // Integracion de Euler y proyeccion a [A_MIN, A_MAX] (no modificar)
    a_hat_ += Ts_ * a_dot;
    a_hat_ = a_hat_.cwiseMax(A_MIN).cwiseMin(A_MAX);
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

    // 2. Captura posicion inicial (primer tick)
    if (!q_initial_captured_) {
      q_initial_ = q;
      ret_.setHome(q);   // pose articular a la que vuelve el retorno final
      q_initial_captured_ = true;
      RCLCPP_INFO(get_logger(), "q_inicial=[%.3f %.3f %.3f %.3f] rad",
        q(0), q(1), q(2), q(3));
    }

    // 3. Referencia: transicion quintica C2 hacia q_d(0), seguimiento, y
    //    retorno articular final + pausa en reposo
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
      if (!ret_.started()) {
        ret_.start(q);
        RCLCPP_INFO(get_logger(), "Iniciando retorno articular a q_inicial (t=%.1fs, dur=%.1fs)",
          t, RETURN_TIME_S);
      }
      ref = ret_.reference(t - (T_TRANS + t_run_));
    }

    // 4. Verificar limites: referencia y estado medido
    for (int i = 0; i < NUM_JOINTS; ++i) {
      if (ref.q(i) < hw_.limits.joint_lower(i) + 0.02 || ref.q(i) > hw_.limits.joint_upper(i) - 0.02) {
        emergency_stop("Referencia fuera de limites articulares");
        return;
      }
      if (q(i) < hw_.limits.joint_lower(i) || q(i) > hw_.limits.joint_upper(i)) {
        emergency_stop("Articulacion " + std::to_string(i+1) + " fuera de limites: "
                       + std::to_string(q(i)) + " rad");
        return;
      }
    }

    // 5. Ley de control — usa dq_hat_ para reducir el ruido de encoder. En el
    //    retorno final el par lo da el retorno articular a q_inicial (la ley
    //    MRAC se evalua igual, pero su par se descarta).
    Vec4 s_q;
    MatY Y;
    const Vec4 tau_mrac  = compute_torque(q, dq_hat_, ref, s_q, Y);
    const Vec4 tau_unsat = returning ? Vec4(ret_.torque(model_, q, dq_hat_, ref)) : tau_mrac;
    const Vec4 tau = tau_unsat.cwiseMin(hw_.limits.tau_max).cwiseMax(-hw_.limits.tau_max);
    std::array<bool, NUM_JOINTS> clipped{};
    const auto cur_cmd = omx::torqueToCurrent(hw_, tau, dq_hat_, ref.dq, &clipped);
    const bool saturated = (tau_unsat.cwiseAbs().maxCoeff() >= hw_.limits.tau_max)
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

    // 7b. Watchdog de sobrecarga: EMA (tau ~1.5 s) de |cmd|/limite por junta
    {
      const double k_ema = Ts_ / 1.5;
      for (int i = 0; i < NUM_JOINTS; ++i) {
        const double duty = std::abs(static_cast<double>(cur_cmd[i]))
                          / static_cast<double>(hw_.limits.current_cmd_limit[i]);
        duty_ema_[i] = (1.0 - k_ema) * duty_ema_[i] + k_ema * duty;
        if (duty_ema_[i] > 0.80) {
          emergency_stop("Sobrecarga sostenida en J" + std::to_string(i + 1)
                         + " (EMA corriente " + std::to_string(duty_ema_[i])
                         + " del limite)");
          return;
        }
      }
    }

    // 8. Adaptacion: en la transicion y el seguimiento, sin saturacion y cerca
    //    de la referencia (compuerta ADAPT_EMAX)
    const bool near_ref = (q - ref.q).cwiseAbs().maxCoeff() <= ADAPT_EMAX;
    if (adaptive_ && (in_trans || in_tracking) && !saturated && near_ref) {
      update_parameters(Y, s_q);
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

    // 10. CSV (transicion inicial y seguimiento; excluye el retorno final y la
    //     pausa en reposo. Los scripts descartan t < T_TRANS)
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
           << ',' << a_hat_[0]  << ',' << a_hat_[1]  << ',' << a_hat_[2]
           << ',' << a_hat_[3]  << ',' << a_hat_[4]  << ',' << a_hat_[5]  << ',' << a_hat_[6]
           << ',' << a_hat_[7]  << ',' << a_hat_[8]  << ',' << a_hat_[9]  << ',' << a_hat_[10] << ',' << a_hat_[11]
           << '\n';
    }

    // 11. Log periodico por consola (~1 s)
    if (++log_cnt_ % static_cast<int>(std::lround(loop_rate_hz_)) == 0) {
      if (csv_.is_open()) csv_.flush();
      const char* phase = in_trans    ? "TRANS" :
                          in_tracking ? "TRAY " :
                          (t < T_TRANS + t_run_ + RETURN_TIME_S) ? "RET  " : "HOME ";
      RCLCPP_INFO(get_logger(),
        "[%s] t=%.2fs |s|=%.3f |e|=%.4f rad  alpha=[%.3f %.3f %.3f] dm=%.3f dmcx=%.4f "
        "Fv1r=%.3f Fcr=[%.3f %.3f %.3f %.3f] i=[%d %d %d %d]",
        phase, t, s_q.norm(), (q - ref.q).norm(),
        a_hat_[0], a_hat_[1], a_hat_[2], a_hat_[3], a_hat_[4],
        a_hat_[7], a_hat_[8], a_hat_[9], a_hat_[10], a_hat_[11],
        cur_cmd[0], cur_cmd[1], cur_cmd[2], cur_cmd[3]);
    }

    // 12. Deteccion de overrun: si el ciclo (bus + control) no cabe en Ts,
    //     el lazo real corre mas lento de lo configurado
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
  std::array<int, NUM_JOINTS> jid_;
  std::array<Vec10, NUM_JOINTS> pi_nom_;

  std::string port_name_;
  bool        adaptive_;
  double      gain_scale_, t_run_, adapt_dz_;
  std::string mode_str_;

  Vec12 a_prior_;   // [1,1,1, 0...0] — carga y friccion residual parten de cero
  Vec12 a_hat_;     // estimacion actual [alpha1..3, theta_load, Fv1_res, Fc_res]
  Vec12 gamma_;     // GAMMA * gamma_scale

  omx::HwConfig hw_;   // modelo del motor, calibracion y limites (YAML del motor)

  double ab_alpha_, ab_beta_;
  double loop_rate_hz_{200.0};
  double Ts_{0.005};
  Vec4   q_hat_, dq_hat_;
  bool   ab_initialized_{false};
  Vec4   duty_ema_ = Vec4::Zero();  // watchdog: EMA de |cmd|/limite por junta

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

// ============================================================
// main
// ============================================================

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  const rclcpp::NodeOptions opts = omx::motorParamsNodeOptions(argc, argv, "hw_mrac_joint_12p_node");

  try {
    auto node = std::make_shared<HWMRACJoint12pNode>(opts);
    if (rclcpp::ok()) {   // false: Ctrl+C durante la espera de la posicion inicial
      rclcpp::spin(node);
    }
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("hw_mrac_joint_12p_node"), "Excepcion: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
