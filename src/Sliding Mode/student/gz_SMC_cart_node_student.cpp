// ============================================================================
//  gz_SMC_cart_node_student.cpp
//  Control por Modo Deslizante en espacio cartesiano — OpenMANIPULATOR-X (Gazebo)
//  [Archivo base para estudiantes — SMC, Actividad 2]
//
//  Salida cartesiana reducida:
//    y = [x_ee, y_ee, z_ee, phi]^T
//    phi = q2 + q3 + q4  (orientacion reducida del gripper)
//
//  Cinematica diferencial:
//    ydot  = J_y(q) * qdot
//    yddot = J_y(q) * qddot + Jydot(q,qdot) * qdot
//
//  Pseudo-inversa amortiguada DLS:
//    J_y^# = J_y^T (J_y J_y^T + lambda_J^2 * I)^{-1}
//
//  Feedforward de friccion del URDF:
//    tau_fric = Fv·dq + Fc·tanh(dq_d/eps)
//      Fv, Fc : <dynamics damping/friction> de joint1..4, leidos del modelo
//               Pinocchio — los mismos valores que simula Gazebo con
//               damping_scale = 1.0 y friction_scale = 1.0.
//      dq_d   : velocidad articular DESEADA = J_y^# * ydot_d (senal limpia;
//               con dq medida el tanh conmutaria con el ruido cerca de cero).
//
//  Transicion inicial: quintica generalizada de duracion T_TRANS que empalma
//  posicion, velocidad Y aceleracion con la trayectoria de referencia en t=0
//  (empalme C2: sin escalon de ydot_d/yddot_d al iniciar la fase SMC).
//
//  Fases (con t_run = 0 no hay limite ni retorno):
//    [0, T_TRANS)                     transicion quintica hasta Y_START
//    [T_TRANS, T_TRANS+t_run)         seguimiento de la trayectoria
//    [T_TRANS+t_run, +RETURN_TIME_S)  retorno quintico al reposo en la pose inicial
//    [..., +HOLD_TIME_S)              pausa en reposo; luego torque cero y fin
//  El retorno deja el robot en la pose de partida, asi que se puede volver a
//  ejecutar el nodo sobre el mismo Gazebo sin relanzarlo. Usa la misma ley de
//  control (SECCION 5), con la referencia del retorno.
//
//  Lazo de control: 200 Hz de tiempo simulado (Ts = 5 ms). El nodo usa el reloj
//  de Gazebo (use_sim_time=true): t_run y el CSV quedan en segundos simulados.
//
//  Termino integral de la ley de alcance (parametro integral):
//    dz/dt = K_I*s,   |z_i| <= Z_MAX_i,   z(0) = 0
//    integral:=false -> z = 0 (SMC clasico)
//    integral:=true  -> z integra desde la transicion inicial y se congela
//                       mientras el torque satura.
//    La actualizacion de z_ y el anti-windup ya estan implementados: en la ley
//    solo se resta z_ (ver SECCION de la ley de control).
//
//  Funciones de conmutacion (aplicadas elemento a elemento):
//    "sat"   ->  rho(s) = sat(s / phi)     phi: capa limite
//    "sign"  ->  rho(s) = sign(s)
//
//  Suscriptor : /joint_states                    (sensor_msgs/JointState)
//  Publicador : /arm_effort_controller/commands   (std_msgs/Float64MultiArray)
//
//  Parametros ROS 2:
//    test_num  [int]     1        — identificador del CSV
//    t_run     [double]  0.0      — duracion del seguimiento en segundos (0 = ilimitado, sin retorno); total =
//                                   T_TRANS + t_run + RETURN_TIME_S + HOLD_TIME_S.
//                                   Al terminar, el nodo se cierra solo.
//    rho_func  [string]  "sat"    — funcion de conmutacion: "sat" | "sign"
//    integral  [bool]    false    — activa el termino integral z
//
//  CSV: data/smc/sim/act2/gz_smc_cart_<rho_func>_<test_num>.csv
//  Columnas: t, q1..q4, x,y,z,phi, x_des,y_des,z_des,phi_des,
//            xdot,ydot,zdot,phidot, xdot_des,ydot_des,zdot_des,phidot_des,
//            s1,s2,s3,s4, tau1..tau4, cond_J, z1..z4
//  (Sat% se calcula en MATLAB desde tau con el criterio >= 0.99*tau_max. El
//  nombre no distingue integral: usar un test_num distinto para cada ensayo.)
//
//  Nota: usar sim_init_config.yaml con use_fixed_init: true y escalas nominales
//        mass_inertia/damping/friction = 1.0 (la friccion del feedforward debe
//        coincidir con la de la planta: friction_scale 0.0 NO es valido aqui).
//        El CSV almacena la transicion y el seguimiento, sin retorno ni pausa;
//        los scripts plots_SMC_* descartan t < T_TRANS para las metricas.
//
//  Ejemplos de uso:
//
//    ros2 run open_manipulator_x_torque_control gz_smc_cart_node_student --ros-args -p test_num:=1 -p t_run:=25.0
//
//    ros2 run open_manipulator_x_torque_control gz_smc_cart_node_student --ros-args -p rho_func:=sign -p test_num:=2 -p t_run:=25.0
//
//    ros2 run open_manipulator_x_torque_control gz_smc_cart_node_student --ros-args -p integral:=true -p test_num:=3 -p t_run:=25.0
//
//  ──────────────────────────────────────────────────────────────────────────
//  SECCIONES A COMPLETAR:
//    [1] Trayectoria cartesiana de referencia  →  desiredTrajectory()
//    [2] Vector Y_START                        →  constante Y_START
//    [3] Funciones de conmutacion              →  rho_scalar(), PHI
//    [4] Ganancias SMC                         →  LAMBDA_Y, K_V, K_S, K_I
//    [5] Ley SMC cartesiana (pasos 7-10 y 12)  →  tick()
// ============================================================================

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
#include "std_msgs/msg/float64_multi_array.hpp"

#include <Eigen/Dense>   // Core + descomposiciones (antes llegaban via Pinocchio)
#include <Eigen/SVD>
#include "omx/robot_model.hpp"
#include "omx/ros_utils.hpp"
#include "omx/trajectory.hpp"

using namespace std::chrono_literals;

#ifndef PACKAGE_DATA_DIR
#define PACKAGE_DATA_DIR "."
#endif
#ifndef PACKAGE_URDF_DIR
#define PACKAGE_URDF_DIR "."
#endif

static constexpr double PI      = M_PI;
static constexpr int    NARM    = 4;
static constexpr double TAU_MAX = 1.2;    // [N·m] limite de torque por articulacion
static constexpr double DT      = 0.005;  // [s] periodo de control, 200 Hz simulados (no modificar)
// Suavizado del tanh de Coulomb del feedforward de friccion (no modificar).
// Se alimenta con la velocidad DESEADA (sin ruido): eps pequeno es seguro.
static constexpr double FRIC_EPS = 0.05;  // [rad/s]
// Amortiguamiento DLS (no modificar; aumentar solo si kappa(J) > 100)
static constexpr double LAMBDA_DLS    = 0.01;
static constexpr double LAMBDA_DLS_SQ = LAMBDA_DLS * LAMBDA_DLS;

// Frame del efector final (no modificar)
static constexpr char EFF_FRAME[] = "end_effector_link";

// Duracion de la transicion quintica desde y0_ hasta Y_START (no modificar)
static constexpr double T_TRANS = 3.0;   // [s]
// Duraciones del retorno final a la pose inicial y de la pausa en reposo antes
// de soltar el torque (no modificar)
static constexpr double RETURN_TIME_S = 3.0;   // [s]
static constexpr double HOLD_TIME_S   = 0.5;   // [s]

// ─────────────────────────────────────────────────────────────────────────────
//  [SECCION 2] Vector Y_START — COMPLETAR
//
//  Y_START es la pose cartesiana [x, y, z, phi] de inicio de la trayectoria
//  de referencia, obtenida de la cinematica directa FK evaluada en q_d(0):
//    Y_START = FK(q_d(0))
//
//  Formato: {x [m], y [m], z [m], phi [rad]}
// ─────────────────────────────────────────────────────────────────────────────
static const Eigen::Vector4d Y_START {0.0, 0.0, 0.0, 0.0};  // COMPLETAR

// ═══════════════════════════════════════════════════════════════════════════
//  [SECCION 4] GANANCIAS SMC CARTESIANO — COMPLETAR
//  Ajustar los valores para cada salida cartesiana [x, y, z, phi]
//
//  Rangos recomendados (lazo discreto a 200 Hz, Ts = 5 ms):
//    LAMBDA_Y : 5   – 35    [1/s]           (ancho de banda de la superficie)
//    K_V      : 5   – 50                    (alcance exponencial; proporcional a s)
//    K_S      : 0.1 – 15    [m/s² | rad/s²] (ganancia de conmutacion)
//    K_I      : 20  – 400   [1/s²]          (integral de s; solo con integral:=true)
//
//  Criterio:
//   · K_S debe superar la perturbacion en aceleracion d_y = J_y*M^-1*dtau de la
//     incertidumbre que se quiere dominar (±20 % de masa). El canal phi recibe
//     la mayor perturbacion por la inercia efectiva diminuta de joint4.
//   · PHI es unica: sus unidades mezclan m/s y rad/s. La pendiente de cada
//     canal se reparte entre K_V y K_S/PHI.
//   · Pendiente dentro de la capa: K_eff = K_V + K_S/PHI <= 75 [1/s]. Fuera de
//     la capa solo queda K_V: conviene que no sea pequeno.
//   · Sin integral, el error DC es ~ d_y/(Lambda*K_eff).
//   · Dentro de la capa la ley equivale a un PID:
//       kd = Lambda + K_eff    kp = Lambda*K_eff + K_I    ki = Lambda*K_I
// ═══════════════════════════════════════════════════════════════════════════
static const Eigen::Vector4d LAMBDA_Y = {0.0, 0.0, 0.0, 0.0};  // COMPLETAR
static const Eigen::Vector4d K_V      = {0.0, 0.0, 0.0, 0.0};  // COMPLETAR
static const Eigen::Vector4d K_S      = {0.0, 0.0, 0.0, 0.0};  // COMPLETAR
static const Eigen::Vector4d K_I      = {0.0, 0.0, 0.0, 0.0};  // COMPLETAR
// ═══════════════════════════════════════════════════════════════════════════

// Tope anti-windup del termino integral |z_i| <= Z_MAX_i [m/s² | rad/s²] (no modificar)
static const Eigen::Vector4d Z_MAX = {40.0, 40.0, 60.0, 600.0};

// ── Estructura de referencia cartesiana ──────────────────────────────────────
using omx::CartRef;     // {y, ydot, yddot}, y = [x, y, z, phi]

// ─────────────────────────────────────────────────────────────────────────────
//  [SECCION 1] Trayectoria cartesiana de referencia — COMPLETAR
//
//  Definir la trayectoria como una serie de Fourier (w = 1.0 rad/s):
//    y_d(t)    = a0 + sum_n [ An*cos(n*w*t) + Bn*sin(n*w*t) ]
//    ydot_d(t) = derivada analitica exacta de y_d(t)
//    yddot_d(t)= derivada analitica exacta de ydot_d(t)
//
//  Las variables auxiliares sN = sin(N*t) y cN = cos(N*t) ya estan
//  declaradas para facilitar la escritura de las expresiones.
// ─────────────────────────────────────────────────────────────────────────────
static CartRef desiredTrajectory(double t)
{
  const double s1 = std::sin(t);
  const double c1 = std::cos(t);
  const double s2 = std::sin(2.0 * t);
  const double c2 = std::cos(2.0 * t);
  const double s3 = std::sin(3.0 * t);
  const double c3 = std::cos(3.0 * t);
  const double s4 = std::sin(4.0 * t);
  const double c4 = std::cos(4.0 * t);
  (void)s1; (void)c1; (void)s2; (void)c2;   // suprimir warnings mientras la
  (void)s3; (void)c3; (void)s4; (void)c4;   // seccion esta incompleta

  CartRef ref;

  // COMPLETAR: posicion deseada y_d(t) = [x_d, y_d, z_d, phi_d]
  ref.y <<
      0.0,   // x_d   : COMPLETAR
      0.0,   // y_d   : COMPLETAR
      0.0,   // z_d   : COMPLETAR
      0.0;   // phi_d : COMPLETAR

  // COMPLETAR: velocidad deseada ydot_d(t) (derivada analitica de y_d)
  ref.ydot <<
      0.0,   // xdot_d   : COMPLETAR
      0.0,   // ydot_d   : COMPLETAR
      0.0,   // zdot_d   : COMPLETAR
      0.0;   // phidot_d : COMPLETAR

  // COMPLETAR: aceleracion deseada yddot_d(t) (derivada analitica de ydot_d)
  ref.yddot <<
      0.0,   // xddot_d   : COMPLETAR
      0.0,   // yddot_d   : COMPLETAR
      0.0,   // zddot_d   : COMPLETAR
      0.0;   // phiddot_d : COMPLETAR

  return ref;
}

// ─────────────────────────────────────────────────────────────────────────────
//  [SECCION 3] Funciones de conmutacion — COMPLETAR
//
//  Implementar rho(s) para cada tipo:
//    sat  : rho(s) = clamp(s / phi, -1, 1)        (phi: capa limite)
//    sign : rho(s) = +1 si s > 0,  -1 si s < 0,  0 si s == 0
//
//  Definir tambien PHI, la capa limite que el nodo pasa como phi a sat:
//    PHI : 0.1 – 0.5 [m/s | rad/s], unica para los 4 canales (sus unidades
//          mezclan m/s y rad/s); solo se usa con rho_func:=sat. PHI pequena
//          acerca sat a sign (mas chattering); PHI grande baja K_S/PHI y con
//          ella la rigidez dentro de la capa.
// ─────────────────────────────────────────────────────────────────────────────
static const double PHI = 0.0;   // COMPLETAR [m/s | rad/s]

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
[[maybe_unused]] static Eigen::Vector4d rho_vec(const Eigen::Vector4d & s,
                                RhoFunc func, double phi)
{
  Eigen::Vector4d r;
  for (int i = 0; i < NARM; ++i) {
    r[i] = rho_scalar(s[i], func, phi);
  }
  return r;
}

// ── Nodo principal ────────────────────────────────────────────────────────────
class SMCCartSimNode : public rclcpp::Node
{
public:
  SMCCartSimNode()
  : Node("gz_smc_cart_node"), t_(0.0), y0_initialized_(false)
  {
    // ── Reloj de simulacion: el timer avanza con /clock de Gazebo ────────────
    this->set_parameter(rclcpp::Parameter("use_sim_time", true));

    // ── Parametros ────────────────────────────────────────────────────────────
    this->declare_parameter<int>        ("test_num", 1);
    this->declare_parameter<double>     ("t_run",    0.0);
    this->declare_parameter<std::string>("rho_func", "sat");
    this->declare_parameter<bool>       ("integral", false);

    const int         test_num = this->get_parameter("test_num").as_int();
    t_run_                     = this->get_parameter("t_run").as_double();
    integral_                  = this->get_parameter("integral").as_bool();
    const std::string rho_str  = this->get_parameter("rho_func").as_string();

    if (rho_str == "sign") {
      rho_func_ = RhoFunc::SIGN;
      rho_str_  = "sign";
    } else {
      rho_func_ = RhoFunc::SAT;
      rho_str_  = "sat";
      if (rho_str != "sat") {
        RCLCPP_WARN(this->get_logger(),
          "rho_func '%s' desconocida — usando 'sat'", rho_str.c_str());
      }
    }

    // ── Modelo Pinocchio ──────────────────────────────────────────────────────
    const std::string urdf_path =
      std::string(PACKAGE_URDF_DIR) + "/open_manipulator_x.urdf";
    try {
      model_.load(urdf_path, EFF_FRAME);
    } catch (const std::exception & e) {
      RCLCPP_FATAL(this->get_logger(), "Pinocchio buildModel: %s", e.what());
      throw;
    }
    // Friccion articular del URDF (<dynamics damping/friction> joint1..4):
    // feedforward de viscosa + Coulomb, mismos valores que simula Gazebo
    // con damping_scale = 1.0 y friction_scale = 1.0. (No modificar)
    fric_damping_ = model_.urdfDamping();
    fric_coulomb_ = model_.urdfFriction();

    // PHI se completa en la SECCION 3: sat(s/PHI) necesita PHI > 0
    if (rho_func_ == RhoFunc::SAT && !(PHI > 0.0)) {
      RCLCPP_FATAL(this->get_logger(),
        "PHI = %.3f: completar PHI > 0 en la SECCION 3 (capa limite de sat)", PHI);
      throw std::runtime_error("PHI sin completar");
    }

    RCLCPP_INFO(this->get_logger(),
      "SMC cartesiano — rho=%s  PHI=%.4f  integral=%s  tau_max=%.2f N·m",
      rho_str_.c_str(), PHI, integral_ ? "true" : "false", TAU_MAX);
    RCLCPP_INFO(this->get_logger(),
      "Lambda_y=[%.1f %.1f %.1f %.1f]  Kv=[%.1f %.1f %.1f %.1f]  Ks=[%.1f %.1f %.1f %.1f]",
      LAMBDA_Y[0], LAMBDA_Y[1], LAMBDA_Y[2], LAMBDA_Y[3],
      K_V[0],      K_V[1],      K_V[2],      K_V[3],
      K_S[0],      K_S[1],      K_S[2],      K_S[3]);
    if (integral_) {
      RCLCPP_INFO(this->get_logger(),
        "Ki=[%.1f %.1f %.1f %.1f]  z_max=[%.1f %.1f %.1f %.1f]",
        K_I[0],   K_I[1],   K_I[2],   K_I[3],
        Z_MAX[0], Z_MAX[1], Z_MAX[2], Z_MAX[3]);
    }
    RCLCPP_INFO(this->get_logger(),
      "lambda_DLS=%.3f  T_trans=%.1f s  Friccion URDF — "
      "Fv=[%.4f %.4f %.4f %.4f] N·m·s/rad  Fc=[%.4f %.4f %.4f %.4f] N·m",
      LAMBDA_DLS, T_TRANS,
      fric_damping_[0], fric_damping_[1], fric_damping_[2], fric_damping_[3],
      fric_coulomb_[0], fric_coulomb_[1], fric_coulomb_[2], fric_coulomb_[3]);
    RCLCPP_INFO(this->get_logger(),
      "Y_start=[%.3f %.3f %.3f %.3f]",
      Y_START[0], Y_START[1], Y_START[2], Y_START[3]);
    if (t_run_ > 0.0) {
      RCLCPP_INFO(this->get_logger(),
        "t_run (seguimiento) = %.1f s  |  total = %.1f s (T_trans=%.1f + t_run=%.1f + retorno=%.1f + pausa=%.1f)",
        t_run_, T_TRANS + t_run_ + RETURN_TIME_S + HOLD_TIME_S, T_TRANS, t_run_, RETURN_TIME_S, HOLD_TIME_S);
    } else {
      RCLCPP_INFO(this->get_logger(), "t_run = ilimitado");
    }

    open_csv(test_num);

    torque_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/arm_effort_controller/commands", 10);

    timer_ = rclcpp::create_timer(
      this, this->get_clock(), rclcpp::Duration::from_seconds(DT),
      [this]() { tick(); });
  }

  ~SMCCartSimNode()
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
    csv_path_ = std::string(PACKAGE_DATA_DIR) + "/smc/sim/act2/gz_smc_cart_"
                + rho_str_ + "_" + std::to_string(test_num) + ".csv";
    omx::openCsv(csv_, csv_path_,
      "t,"
      "q1,q2,q3,q4,"
      "x,y,z,phi,"
      "x_des,y_des,z_des,phi_des,"
      "xdot,ydot,zdot,phidot,"
      "xdot_des,ydot_des,zdot_des,phidot_des,"
      "s1,s2,s3,s4,"
      "tau1,tau2,tau3,tau4,"
      "cond_J,"
      "z1,z2,z3,z4",
      this->get_logger());
  }

  // Termino integral de la ley de alcance (infraestructura, no modificar):
  // dz/dt = K_I*s, con tope |z_i| <= Z_MAX_i. Vale cero con integral:=false y
  // se congela si el tick anterior saturo algun torque.
  void update_integral(const Eigen::Vector4d & s)
  {
    if (integral_ && !tau_saturated_prev_) {
      z_ += DT * K_I.cwiseProduct(s);
      z_ = z_.cwiseMin(Z_MAX).cwiseMax(-Z_MAX);
    }
  }

  // ── Tick de control a 200 Hz ──────────────────────────────────────────────
  void tick()
  {
    if (!js_.ready()) { return; }

    // 1. Leer estados articulares
    Eigen::Vector4d q, dq;
    js_.read(q, dq);

    // 2. Cinematica directa + Jacobiano reducido 4x4
    //    Filas: {vx, vy, vz, dphi/dq}  Columnas: {j1..j4}
    const auto ee = model_.eeKinematics(q, dq);   // p, J lineal y J̇·q̇ del efector

    const Eigen::Vector3d p_ee = ee.p;
    const double phi_ee = q[1] + q[2] + q[3];  // orientacion reducida analitica

    Eigen::Matrix4d J4;
    J4.topRows<3>() = ee.J;
    J4.row(3) << 0.0, 1.0, 1.0, 1.0;  // dphi/dq = [0,1,1,1]

    // Velocidad cartesiana actual: ydot = J4 * qdot
    const Eigen::Vector4d ydot = J4 * dq;

    // 3. Termino de bias: Jdot*qdot (via aceleracion clasica con qddot=0)
    Eigen::Vector4d jdqd;
    jdqd << ee.dJdq, 0.0;  // Jdot_phi * qdot = 0

    // 4. Dinamica nominal: M(q) y Phi(q,dq) = C*dq + g
    const Eigen::Matrix4d M4   = model_.massMatrix(q);
    const Eigen::Vector4d nle4 = model_.nonlinearEffects(q, dq);

    // 5. Pose cartesiana actual + captura de condicion inicial
    const Eigen::Vector4d y {p_ee[0], p_ee[1], p_ee[2], phi_ee};

    if (!y0_initialized_) {
      y0_ = y;
      y0_initialized_ = true;
      RCLCPP_INFO(this->get_logger(),
        "Pose inicial capturada: x=%.3f  y=%.3f  z=%.3f  phi=%.3f rad",
        y0_[0], y0_[1], y0_[2], y0_[3]);
    }

    // 6. Referencia segun fase (transicion quintica → trayectoria SMC → retorno).
    //    La transicion empalma en C2 con la trayectoria: posicion Y_START
    //    (= FK(q_d(0))) y velocidad/aceleracion de la serie de Fourier en t=0.
    //    Al cumplirse t_run, retorno quintico a la pose inicial y pausa en
    //    reposo. (Infraestructura, no modificar)
    CartRef goal0 = desiredTrajectory(0.0);
    goal0.y = Y_START;
    const bool in_trans    = (t_ < T_TRANS);
    const bool in_tracking = !in_trans && (t_run_ <= 0.0 || t_ < T_TRANS + t_run_);
    CartRef ref;
    if (in_trans) {
      ref = omx::quinticTransition(t_, T_TRANS, y0_, goal0);
    } else if (in_tracking) {
      ref = desiredTrajectory(t_ - T_TRANS);
    } else if (t_ < T_TRANS + t_run_ + RETURN_TIME_S) {
      if (!return_logged_) {
        RCLCPP_INFO(this->get_logger(),
          "Iniciando retorno quintico a y_inicial (t=%.1fs, dur=%.1fs)", t_, RETURN_TIME_S);
        return_logged_ = true;
      }
      ref = omx::quinticReturn(t_ - (T_TRANS + t_run_), RETURN_TIME_S,
                               desiredTrajectory(t_run_), y0_);
    } else {
      ref.y = y0_;
      ref.ydot.setZero();
      ref.yddot.setZero();
    }

    // 6b. Feedforward de friccion del URDF (infraestructura, no modificar):
    //     viscosa con dq medida + Coulomb con la velocidad articular DESEADA,
    //     mapeada de ydot_d via la pseudo-inversa DLS
    Eigen::Vector4d tau_fric;
    {
      const Eigen::Matrix4d A_fric =
        J4 * J4.transpose() + LAMBDA_DLS_SQ * Eigen::Matrix4d::Identity();
      const Eigen::Vector4d dq_des =
        J4.transpose() * A_fric.ldlt().solve(ref.ydot);
      for (int i = 0; i < NARM; ++i) {
        tau_fric[i] = fric_damping_[i] * dq[i]
                    + fric_coulomb_[i] * std::tanh(dq_des[i] / FRIC_EPS);
      }
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  [SECCION 5] Ley SMC cartesiana — COMPLETAR (pasos 7-10 y 12)
    //  Disponible: y, ydot, ref.y, ref.ydot, ref.yddot, J4, jdqd, M4, nle4,
    //              tau_fric, z_ (4x1, termino integral), rho_func_, PHI
    // ─────────────────────────────────────────────────────────────────────────

    // 7. Errores cartesianos:
    const Eigen::Vector4d e_y    = Eigen::Vector4d::Zero();
    const Eigen::Vector4d edot_y = Eigen::Vector4d::Zero();

    // 8. Superficie deslizante cartesiana: 
    // COMPLETAR
    const Eigen::Vector4d s_y = Eigen::Vector4d::Zero();

    // 9. Funcion de conmutacion: rho = rho_vec(s_y, ...)
    // COMPLETAR
    const Eigen::Vector4d rho = Eigen::Vector4d::Zero();

    // Termino integral z_ (infraestructura, no modificar):
    update_integral(s_y);

    // 10. Aceleracion cartesiana virtual (restar z_; con integral:=false vale 0):
    // COMPLETAR
    const Eigen::Vector4d v_y = Eigen::Vector4d::Zero();

    // 11. Aceleracion articular via pseudo-inversa DLS (no modificar):
    //     qddot = J_y^T (J_y J_y^T + lambda^2 I)^{-1} (v_y - Jydot*qdot)
    const Eigen::Matrix4d A_dls =
      J4 * J4.transpose() + LAMBDA_DLS_SQ * Eigen::Matrix4d::Identity();
    const Eigen::Vector4d qddot = J4.transpose() * A_dls.ldlt().solve(v_y - jdqd);

    // 12. Torque nominal y saturado:
    // COMPLETAR
    const Eigen::Vector4d tau     = Eigen::Vector4d::Zero();
    const Eigen::Vector4d tau_sat = tau.cwiseMin(TAU_MAX).cwiseMax(-TAU_MAX);
    tau_saturated_prev_ = (tau.cwiseAbs().maxCoeff() >= TAU_MAX);   // anti-windup (no modificar)

    // Suprimir warnings mientras la seccion esta incompleta (eliminar al completarla):
    (void)M4; (void)nle4; (void)tau_fric; (void)edot_y; (void)rho; (void)qddot;
    // ─────────────────────────────────────────────────────────────────────────

    // 13. Condicionamiento del Jacobiano (no modificar)
    Eigen::JacobiSVD<Eigen::Matrix4d> svd(J4);
    const double sigma_min = svd.singularValues()(NARM - 1);
    const double cond_J    = (sigma_min > 1e-10) ?
      svd.singularValues()(0) / sigma_min : 1e10;

    // 14. Publicar torques
    std_msgs::msg::Float64MultiArray cmd;
    cmd.data.assign(tau_sat.data(), tau_sat.data() + NARM);
    torque_pub_->publish(cmd);

    const char * phase = in_trans    ? "TRANS" :
                         in_tracking ? "TRAY " :
                         (t_ < T_TRANS + t_run_ + RETURN_TIME_S) ? "RET  " : "HOME ";
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "[%s] t=%.2fs  |s|=%.4f  |e|=%.4f m  kJ=%.1f  tau=[%.3f %.3f %.3f %.3f] N·m",
      phase, t_, s_y.norm(), e_y.norm(), cond_J,
      tau_sat[0], tau_sat[1], tau_sat[2], tau_sat[3]);

    // 15. Registro CSV (transicion y seguimiento, sin retorno ni pausa)
    if ((in_trans || in_tracking) && csv_.is_open()) {
      csv_ << std::fixed << std::setprecision(6)
           << t_
           << ',' << q[0]           << ',' << q[1]           << ',' << q[2]           << ',' << q[3]
           << ',' << y[0]           << ',' << y[1]           << ',' << y[2]           << ',' << y[3]
           << ',' << ref.y[0]       << ',' << ref.y[1]       << ',' << ref.y[2]       << ',' << ref.y[3]
           << ',' << ydot[0]        << ',' << ydot[1]        << ',' << ydot[2]        << ',' << ydot[3]
           << ',' << ref.ydot[0]    << ',' << ref.ydot[1]    << ',' << ref.ydot[2]    << ',' << ref.ydot[3]
           << ',' << s_y[0]         << ',' << s_y[1]         << ',' << s_y[2]         << ',' << s_y[3]
           << ',' << tau_sat[0]     << ',' << tau_sat[1]     << ',' << tau_sat[2]     << ',' << tau_sat[3]
           << ',' << cond_J
           << ',' << z_[0]          << ',' << z_[1]          << ',' << z_[2]          << ',' << z_[3]
           << '\n';
    }

    t_ += DT;

    // Fin: tras el retorno y la pausa en reposo se suelta el torque (el robot
    // queda en la pose de partida, listo para otra ejecucion)
    if (t_run_ > 0.0 && t_ >= T_TRANS + t_run_ + RETURN_TIME_S + HOLD_TIME_S) {
      RCLCPP_INFO(this->get_logger(),
        "Retorno a y_inicial completado (t=%.1f s). Deteniendo control.", t_);
      std_msgs::msg::Float64MultiArray zero;
      zero.data.assign(NARM, 0.0);
      torque_pub_->publish(zero);
      if (csv_.is_open()) {
        csv_.close();
        RCLCPP_INFO(this->get_logger(), "CSV cerrado: %s", csv_path_.c_str());
      }
      timer_->cancel();
      // Fin de la ejecucion sin Ctrl+C: breve espera para que el torque cero
      // llegue al controlador de Gazebo y luego se cierra el proceso
      rclcpp::sleep_for(100ms);
      rclcpp::shutdown();
    }
  }

  // ── Miembros ───────────────────────────────────────────────────────────────
  omx::RobotModel model_;

  Eigen::Vector4d fric_damping_;   // Fv del URDF [N·m·s/rad]
  Eigen::Vector4d fric_coulomb_;   // Fc del URDF [N·m]

  double      t_;
  double      t_run_;
  RhoFunc     rho_func_;
  std::string rho_str_;

  Eigen::Vector4d y0_;
  bool            y0_initialized_;

  bool            integral_ = false;
  Eigen::Vector4d z_ {0.0, 0.0, 0.0, 0.0};   // termino integral [m/s² | rad/s²]
  bool            tau_saturated_prev_ = false;
  bool            return_logged_ = false;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_pub_;
  omx::JointStateListener js_{*this};   // /joint_states (joint1..joint4 por nombre)
  rclcpp::TimerBase::SharedPtr timer_;

  std::ofstream csv_;
  std::string   csv_path_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<SMCCartSimNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("gz_smc_cart_node"), "Excepcion: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
