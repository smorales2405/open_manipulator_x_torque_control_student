// ============================================================================
//  gz_MRAC_joint_12p_node_student.cpp
//  Control Adaptativo (MRAC / Slotine-Li) articular — 12 PARAMETROS
//  OpenMANIPULATOR-X (Gazebo)
//  [Archivo base para estudiantes — MRAC, simulacion]
//
//  Modelo dinamico nominal con friccion (Guia SMC, Sec. 6.1):
//    M(q)*ddq + C(q,dq)*dq + g(q) + tau_fric = tau
//    tau_fric = Fv·dq + Fc·tanh(dq_d/eps)
//      Fv, Fc nominales: <dynamics damping/friction> de joint1..4, leidos del
//      modelo Pinocchio (los mismos valores que simula Gazebo con
//      damping_scale = 1.0 y friction_scale = 1.0).
//
//  Parametros adaptados (12):
//    a_hat = [ alpha1..alpha3 , dm, dmcx, dmcy, dmcz , Fv1 , Fc1..Fc4 ]^T
//      alpha_k   : escala de inercia del cuerpo movido por jointk, nominal 1.0
//      theta_load: exceso de masa [kg] y de primer momento [kg·m] del ultimo
//                  cuerpo (link5 + lo que sujete el gripper), nominal 0
//      Fv1       : friccion viscosa de joint1 [N·m·s/rad]
//      Fc_j      : friccion de Coulomb de jointj [N·m]
//    El prior a_prior, la fuga sigma, las cotas de los parametros y las
//    columnas de la carga en el regresor ya estan implementados.
//
//  Referencias de Slotine-Li:
//    e_q = q - q_d,   e_dq = dq - dq_d,   s = e_dq + Lambda*e_q
//    dq_r = dq_d - Lambda*e_q,   ddq_r = ddq_d - Lambda*e_dq
//
//  Torque nominal de Slotine-Li con la friccion nominal del URDF:
//    tau_nom = M(q)*ddq_r + C(q,dq)*dq_r + g(q) + Fv0.*dq + Fc0.*tanh(dq_d/eps)
//  Regresor Y (4x12), en el orden de a_hat: la dinamica es lineal en los
//  parametros, y Y*(a - a_prior) es el cambio de tau_nom cuando los
//  parametros pasan de a_prior a a.
//  Ley de control:
//    tau = tau_nom + Y*(a_hat - a_prior) - M(q)*(K_V*s + K_S*rho(s))
//
//  Ley de adaptacion: a_hat_dot = -Gamma .* ( Y^T*s + sigma .* (a_hat - a_prior) ).
//  La integracion, la proyeccion a las cotas y el congelamiento cuando el
//  torque satura (anti-windup) ya estan implementados. Con adaptive:=false,
//  a_hat queda en a_prior.
//
//  Funcion de conmutacion (aplicada elemento a elemento; ya implementada):
//    rho(s) = sat(s / phi)        phi: capa limite PHI [rad/s]
//
//  Transicion inicial: quintica de duracion T_TRANS desde la pose medida al
//  arrancar hasta q_d(0), empalmando posicion, velocidad y aceleracion. El
//  CSV registra la transicion y el seguimiento, sin retorno ni pausa; los
//  scripts descartan t < T_TRANS para que las metricas sean solo de
//  seguimiento. La adaptacion esta activa en la transicion y el seguimiento.
//
//  Fases (con t_run = 0 no hay limite ni retorno):
//    [0, T_TRANS)                     transicion quintica hasta q_d(0)
//    [T_TRANS, T_TRANS+t_run)         seguimiento de la trayectoria
//    [T_TRANS+t_run, +RETURN_TIME_S)  retorno quintico al reposo en la pose inicial
//    [..., +HOLD_TIME_S)              pausa en reposo; luego torque cero y fin
//  El retorno usa la misma ley con la adaptacion congelada y deja el robot en
//  la pose de partida.
//
//  Lazo de control: 200 Hz de tiempo simulado (Ts = 5 ms). El nodo usa el reloj
//  de Gazebo (use_sim_time=true): t_run y el CSV quedan en segundos simulados.
//
//  Suscriptor : /joint_states                    (sensor_msgs/JointState)
//  Publicador : /arm_effort_controller/commands   (std_msgs/Float64MultiArray)
//
//  Parametros ROS 2 (--ros-args -p nombre:=valor):
//    test_num        [int]     1     — identificador del CSV generado
//    t_run           [double]  0.0   — duracion del seguimiento en segundos
//                                      (0 = ilimitado, sin retorno); total =
//                                      T_TRANS + t_run + RETURN_TIME_S + HOLD_TIME_S.
//                                      Al terminar, el nodo se cierra solo.
//    adaptive        [bool]    true  — true: adapta | false: a_hat fijo en a_prior
//
//  CSV generado: data/mrac/sim/mrac12p/gz_mrac_joint_12p_<modo>_<test_num>.csv
//    <modo> = "adaptive" | "fixed"
//  Columnas: t, q1..q4, dq1..dq4, q1_des..q4_des, dq1_des..dq4_des,
//            s1..s4, tau1..tau4,
//            a1_hat..a3_hat, dm_hat, dmcx_hat, dmcy_hat, dmcz_hat,
//            fv1_hat, fc1_hat..fc4_hat
//
//  Ejemplos de uso:
//
//    ros2 run open_manipulator_x_torque_control gz_mrac_joint_12p_node_student --ros-args -p adaptive:=false -p test_num:=1 -p t_run:=25.0
//
//    ros2 run open_manipulator_x_torque_control gz_mrac_joint_12p_node_student --ros-args -p test_num:=2 -p t_run:=25.0
//
//  ──────────────────────────────────────────────────────────────────────────
//  SECCIONES A COMPLETAR:
//    [1] Trayectoria articular de referencia  →  desiredTrajectory()
//    [2] Modelo nominal y regresor            →  nominalTorque(), regressor()
//    [3] Ganancias MRAC y tasas Gamma         →  LAMBDA_Q, K_V, K_S, PHI, GAMMA_*
//    [4] Ley de control y adaptacion          →  tick()
//  Al arrancar, el nodo verifica la SECCION 2 contra la dinamica de Pinocchio
//  y se detiene con un informe por bloque si algo no coincide.
// ============================================================================

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

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include <Eigen/Core>
#include "omx/mrac_check.hpp"
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
static constexpr int    NARM    = 4;       // articulaciones controladas
static constexpr int    NP      = 12;      // parametros: [alpha1..3, theta_load(4), Fv1, Fc1..4]
static constexpr double TAU_MAX = 1.2;     // [N·m] limite de torque por articulacion
static constexpr double DT      = 0.005;   // [s] periodo de control, 200 Hz simulados (no modificar)
// Suavizado del tanh de Coulomb del feedforward de friccion (no modificar).
// Se alimenta con la velocidad DESEADA (sin ruido): eps pequeno es seguro.
static constexpr double FRIC_EPS = 0.05;   // [rad/s]

// Duracion de la transicion quintica desde la pose medida hasta q_d(0)
// (no modificar)
static constexpr double T_TRANS = 3.0;   // [s]
// Duraciones del retorno final a la pose inicial y de la pausa en reposo antes
// de soltar el torque (no modificar)
static constexpr double RETURN_TIME_S = 3.0;   // [s]
static constexpr double HOLD_TIME_S   = 0.5;   // [s]

// Indices de bloque dentro de a_hat (no modificar):
static constexpr int IDX_ALPHA = 0;   // [0..2]  alpha1..alpha3
static constexpr int IDX_LOAD  = 3;   // [3..6]  dm, dmcx, dmcy, dmcz (exceso del link5)
static constexpr int IDX_FV1   = 7;   // [7]     Fv1
static constexpr int IDX_FC    = 8;   // [8..11] Fc1..Fc4

using Vec4  = Eigen::Vector4d;
using Vec12 = Eigen::Matrix<double, NP, 1>;
using Vec10 = Eigen::Matrix<double, 10, 1>;
using MatY  = Eigen::Matrix<double, NARM, NP>;

// ═══════════════════════════════════════════════════════════════════════════
//  [SECCION 3] GANANCIAS MRAC Y TASAS DE ADAPTACION — COMPLETAR
//  Ajustar los valores para cada articulacion [joint1, joint2, joint3, joint4]
//
//  Rangos recomendados (lazo discreto a 200 Hz, Ts = 5 ms):
//    LAMBDA_Q : 5  – 30    [1/s]     (ancho de banda de la superficie)
//    K_V      : 5  – 50              (alcance exponencial; proporcional a s)
//    K_S      : 1  – 40    [rad/s²]  (ganancia de conmutacion)
//    PHI      : 0.2 – 1.0  [rad/s]   (capa limite de sat, comun a las 4 articulaciones)
//
//  Tasas de adaptacion Gamma, una por bloque de parametros (todas > 0):
//    GAMMA_ALPHA : 0.1   – 0.5     (alpha1..alpha3)
//    GAMMA_DM    : 0.02  – 0.1     (dm, masa de la carga)
//    GAMMA_MC    : 0.001 – 0.005   (dmcx, dmcy, dmcz, primer momento de la carga)
//    GAMMA_FV    : 0.5   – 3.0     (Fv1)
//    GAMMA_FC    : 0.5   – 2.0     (Fc1..Fc4)
//  Con Gamma baja los parametros convergen lento; con Gamma alta el torque
//  oscila y los parametros van de una cota a la otra.
//
// ═══════════════════════════════════════════════════════════════════════════
static const Vec4 LAMBDA_Q = {0.0, 0.0, 0.0, 0.0};  // COMPLETAR
static const Vec4 K_V      = {0.0, 0.0, 0.0, 0.0};  // COMPLETAR
static const Vec4 K_S      = {0.0, 0.0, 0.0, 0.0};  // COMPLETAR
static const double PHI    = 0.0;                   // COMPLETAR [rad/s]

static const double GAMMA_ALPHA = 0.0;   // COMPLETAR
static const double GAMMA_DM    = 0.0;   // COMPLETAR
static const double GAMMA_MC    = 0.0;   // COMPLETAR
static const double GAMMA_FV    = 0.0;   // COMPLETAR
static const double GAMMA_FC    = 0.0;   // COMPLETAR
// ═══════════════════════════════════════════════════════════════════════════

// Tasas de adaptacion por parametro, en el orden de a_hat (no modificar)
static const Vec12 GAMMA =
  (Vec12() << GAMMA_ALPHA, GAMMA_ALPHA, GAMMA_ALPHA,
              GAMMA_DM, GAMMA_MC, GAMMA_MC, GAMMA_MC,
              GAMMA_FV,
              GAMMA_FC, GAMMA_FC, GAMMA_FC, GAMMA_FC).finished();

// Fuga sigma hacia el prior [1/s] y cotas de los parametros, por bloque
// [alpha1..3, dm, dmcx, dmcy, dmcz, Fv1, Fc1..4] (no modificar)
static const Vec12 SIGMA_LEAK =
  (Vec12() << 0.1, 0.1, 0.1,   0.02, 0.02, 0.02, 0.02,   0.1,   0.1, 0.1, 0.1, 0.1).finished();
static const Vec12 A_MIN =
  (Vec12() << 0.80, 0.80, 0.80,   -0.02, -0.005, -0.010, -0.005,   0.0,   0.0, 0.0, 0.0, 0.0).finished();
static const Vec12 A_MAX =
  (Vec12() << 1.25, 1.25, 1.25,    0.20,  0.035,  0.010,  0.020,   0.15,  0.30, 0.30, 0.30, 0.30).finished();

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
  Reference ref;

  (void)t;
  const double w = 1.0;  // [rad/s] — no modificar
  (void)w;

  // COMPLETAR: posicion deseada qi_d(t) = Ai + Bi*sin(w*t)
  ref.q <<
      0.0,   // joint1: COMPLETAR
      0.0,   // joint2: COMPLETAR
      0.0,   // joint3: COMPLETAR
      0.0;   // joint4: COMPLETAR

  // COMPLETAR: velocidad deseada dqi_d(t) = Bi*w*cos(w*t)
  ref.dq <<
      0.0,   // joint1: COMPLETAR
      0.0,   // joint2: COMPLETAR
      0.0,   // joint3: COMPLETAR
      0.0;   // joint4: COMPLETAR

  // COMPLETAR: aceleracion deseada ddqi_d(t) = -Bi*w^2*sin(w*t)
  ref.ddq <<
      0.0,   // joint1: COMPLETAR
      0.0,   // joint2: COMPLETAR
      0.0,   // joint3: COMPLETAR
      0.0;   // joint4: COMPLETAR

  return ref;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Funcion de conmutacion (no modificar):
//    sat : rho(s) = clamp(s / phi, -1, 1)        (phi: capa limite PHI)
// ─────────────────────────────────────────────────────────────────────────────
static double rho_scalar(double s, double phi)
{
  return std::max(-1.0, std::min(1.0, s / phi));
}

// Aplica rho_scalar elemento a elemento sobre el vector s (no modificar)
[[maybe_unused]] static Vec4 rho_vec(const Vec4 & s, double phi)
{
  Vec4 r;
  for (int i = 0; i < NARM; ++i) {
    r[i] = rho_scalar(s[i], phi);
  }
  return r;
}

// ── Nodo principal ────────────────────────────────────────────────────────────
class MRACJoint12pSimNode : public rclcpp::Node
{
public:
  MRACJoint12pSimNode()
  : Node("gz_mrac_joint_12p_node"), t_(0.0)
  {
    // ── Reloj de simulacion: el timer avanza con /clock de Gazebo ────────────
    this->set_parameter(rclcpp::Parameter("use_sim_time", true));

    // ── Parametros ────────────────────────────────────────────────────────────
    this->declare_parameter<int>   ("test_num",       1);
    this->declare_parameter<double>("t_run",          0.0);
    this->declare_parameter<bool>  ("adaptive",       true);

    const int test_num = this->get_parameter("test_num").as_int();
    t_run_             = this->get_parameter("t_run").as_double();
    adaptive_          = this->get_parameter("adaptive").as_bool();

    mode_str_ = adaptive_ ? "adaptive" : "fixed";

    // PHI y Gamma se completan en la SECCION 3: sat(s/PHI) necesita PHI > 0
    // y la adaptacion necesita Gamma > 0
    if (!(PHI > 0.0)) {
      RCLCPP_FATAL(this->get_logger(),
        "PHI = %.3f: completar PHI > 0 en la SECCION 3 (capa limite de sat)", PHI);
      throw std::runtime_error("PHI sin completar");
    }
    if (adaptive_ && !(GAMMA.minCoeff() > 0.0)) {
      RCLCPP_FATAL(this->get_logger(),
        "Gamma sin completar: GAMMA_ALPHA, GAMMA_DM, GAMMA_MC, GAMMA_FV y GAMMA_FC "
        "deben ser > 0 en la SECCION 3 (o ejecutar con adaptive:=false)");
      throw std::runtime_error("Gamma sin completar");
    }

    // ── Modelo Pinocchio (brazo nominal, escala 1.0) ──────────────────────────
    const std::string urdf_path =
      std::string(PACKAGE_URDF_DIR) + "/open_manipulator_x.urdf";
    try {
      model_.load(urdf_path);
    } catch (const std::exception & e) {
      RCLCPP_FATAL(this->get_logger(), "Pinocchio buildModel: %s", e.what());
      throw;
    }

    // Parametros dinamicos nominales del cuerpo movido por cada joint (para
    // el regresor; no modificar)
    for (int i = 0; i < NARM; ++i) {
      const std::string jname = "joint" + std::to_string(i + 1);
      jid_[i]    = model_.jointId(jname);
      pi_nom_[i] = model_.bodyDynamicParameters(jid_[i]);
    }

    // Prior de parametros (= a_hat(0)) con la friccion nominal del URDF.
    // fv_prior_ es el feedforward viscoso FIJO de las 4 juntas; como parametro
    // adaptado solo entra Fv1. theta_load parte de cero. (No modificar.)
    fv_prior_ = model_.urdfDamping();
    a_prior_.setZero();
    a_prior_.segment<3>(IDX_ALPHA).setOnes();
    a_prior_[IDX_FV1]              = fv_prior_[0];
    a_prior_.segment<NARM>(IDX_FC) = model_.urdfFriction();
    a_hat_ = a_prior_;

    // Autoverificacion de la SECCION 2 antes de enviar torque (no modificar)
    check_model();

    RCLCPP_INFO(this->get_logger(),
      "MRAC articular 12p — modo=%s  PHI=%.3f  tau_max=%.2f N·m",
      mode_str_.c_str(), PHI, TAU_MAX);
    RCLCPP_INFO(this->get_logger(),
      "Lambda=[%.1f %.1f %.1f %.1f]  Kv=[%.1f %.1f %.1f %.1f]  Ks=[%.1f %.1f %.1f %.1f]",
      LAMBDA_Q[0], LAMBDA_Q[1], LAMBDA_Q[2], LAMBDA_Q[3],
      K_V[0],      K_V[1],      K_V[2],      K_V[3],
      K_S[0],      K_S[1],      K_S[2],      K_S[3]);
    RCLCPP_INFO(this->get_logger(),
      "Gamma: alpha=%.3f  dm=%.3f  mc=%.4f  Fv1=%.2f  Fc=%.2f",
      GAMMA_ALPHA, GAMMA_DM, GAMMA_MC, GAMMA_FV, GAMMA_FC);
    if (t_run_ > 0.0) {
      RCLCPP_INFO(this->get_logger(),
        "t_run (seguimiento) = %.1f s  |  total = %.1f s (T_trans=%.1f + t_run=%.1f + retorno=%.1f + pausa=%.1f)",
        t_run_, T_TRANS + t_run_ + RETURN_TIME_S + HOLD_TIME_S, T_TRANS, t_run_, RETURN_TIME_S, HOLD_TIME_S);
    } else {
      RCLCPP_INFO(this->get_logger(), "t_run = ilimitado  (T_trans=%.1f s)", T_TRANS);
    }

    open_csv(test_num);

    torque_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/arm_effort_controller/commands", 10);

    timer_ = rclcpp::create_timer(
      this, this->get_clock(), rclcpp::Duration::from_seconds(DT),
      [this]() { tick(); });
  }

  ~MRACJoint12pSimNode()
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
    csv_path_ = std::string(PACKAGE_DATA_DIR) + "/mrac/sim/mrac12p/gz_mrac_joint_12p_"
                + mode_str_ + "_" + std::to_string(test_num) + ".csv";
    omx::openCsv(csv_, csv_path_,
      "t,"
      "q1,q2,q3,q4,"
      "dq1,dq2,dq3,dq4,"
      "q1_des,q2_des,q3_des,q4_des,"
      "dq1_des,dq2_des,dq3_des,dq4_des,"
      "s1,s2,s3,s4,"
      "tau1,tau2,tau3,tau4,"
      "a1_hat,a2_hat,a3_hat,"
      "dm_hat,dmcx_hat,dmcy_hat,dmcz_hat,"
      "fv1_hat,fc1_hat,fc2_hat,fc3_hat,fc4_hat",
      this->get_logger());
  }

  // ═══════════════════════════════════════════════════════════════════════════
  //  [SECCION 2] Modelo nominal y regresor — COMPLETAR
  //  Argumentos: q, dq (medidas), dqr, ddqr (referencias de Slotine-Li) y
  //  tanh_dqd = tanh(dq_d/eps) (4x1, Coulomb con la velocidad deseada).
  //  Al arrancar, check_model() compara estas funciones con la dinamica de
  //  Pinocchio y detiene el nodo si algun bloque no coincide.
  // ═══════════════════════════════════════════════════════════════════════════

  // 2.1 Torque nominal de Slotine-Li con la friccion nominal del URDF:
  //     tau_nom = M(q)*ddq_r + C(q,dq)*dq_r + g(q) + Fv0.*dq + Fc0.*tanh(dq_d/eps)
  Vec4 nominalTorque(const Vec4 & q, const Vec4 & dq, const Vec4 & dqr, const Vec4 & ddqr,
                     const Vec4 & tanh_dqd)
  {
    // Modelo nominal del URDF y friccion nominal (no modificar)
    const Eigen::Matrix4d M = model_.massMatrix(q);           // M(q)
    const Eigen::Matrix4d C = model_.coriolisMatrix(q, dq);   // C(q,dq)
    const Vec4 g   = model_.gravity(q);                        // g(q)
    const Vec4 fv0 = fv_prior_;                                // Fv0 (4x1)
    const Vec4 fc0 = a_prior_.segment<NARM>(IDX_FC);           // Fc0 (4x1)

    // COMPLETAR
    const Vec4 tau_nom = Vec4::Zero();

    // Suprimir warnings mientras la seccion esta incompleta (eliminar al completarla):
    (void)M; (void)C; (void)g; (void)fv0; (void)fc0;
    (void)dq; (void)dqr; (void)ddqr; (void)tanh_dqd;
    return tau_nom;
  }

  // Regresor 4x12: columnas en el orden de a_hat
  //   [alpha1..3 | dm, dmcx, dmcy, dmcz | Fv1 | Fc1..4]
  MatY regressor(const Vec4 & q, const Vec4 & dq, const Vec4 & dqr, const Vec4 & ddqr,
                 const Vec4 & tanh_dqd)
  {
    // Regresor de Pinocchio por cuerpo (no modificar): para el cuerpo movido
    // por joint(k+1), k = 0..3, su aporte al torque es Y_body[k]*pi_k, con
    // pi_k (10x1) = [m, m*cx, m*cy, m*cz, Ixx, Ixy, Iyy, Ixz, Iyz, Izz].
    // pi_nom_[k] tiene los valores nominales de pi_k (URDF).
    const Eigen::MatrixXd Y_SL = model_.slotineLiRegressor(q, dq, dqr, ddqr);
    std::array<Eigen::Matrix<double, NARM, 10>, NARM> Y_body;
    for (int k = 0; k < NARM; ++k) {
      Y_body[k] = Y_SL.block(0, 10 * (jid_[k] - 1), NARM, 10);
    }
    MatY Y = MatY::Zero();

    // Columnas de la carga theta_load = [dm, dmcx, dmcy, dmcz] (no modificar):
    // la carga suma masa y primer momento al ultimo cuerpo (k = 3)
    Y.block<NARM, 4>(0, IDX_LOAD) = Y_body[3].leftCols<4>();

    // 2.2 Columnas de alpha1..alpha3 (cuerpos k = 0..2): alpha_k escala todos
    //     los parametros del cuerpo, pi_k = alpha_k*pi_nom_[k]
    //     Y.col(IDX_ALPHA + k) = ...
    // COMPLETAR

    // 2.3 Columna de Fv1 (friccion viscosa de joint1: Fv1*dq1)
    //     Y(0, IDX_FV1) = ...
    // COMPLETAR

    // 2.4 Columnas de Fc1..Fc4 (Coulomb de cada articulacion: Fc_j*tanh(dq_d,j/eps))
    //     Y(j, IDX_FC + j) = ...
    // COMPLETAR

    // Suprimir warnings mientras la seccion esta incompleta (eliminar al completarla):
    (void)dq; (void)tanh_dqd;
    return Y;
  }
  // ═══════════════════════════════════════════════════════════════════════════

  // Autoverificacion de la SECCION 2 (no modificar): compara nominalTorque() y
  // regressor() con la dinamica de Pinocchio en estados de prueba
  void check_model()
  {
    const auto res = omx::checkMrac12p(
      model_, jid_, omx::MracNominal::SlotineLi, fv_prior_, a_prior_.segment<NARM>(IDX_FC),
      [this](const Vec4 & q, const Vec4 & dq, const Vec4 & dqr, const Vec4 & ddqr,
             const Vec4 & th) { return nominalTorque(q, dq, dqr, ddqr, th); },
      [this](const Vec4 & q, const Vec4 & dq, const Vec4 & dqr, const Vec4 & ddqr,
             const Vec4 & th) { return regressor(q, dq, dqr, ddqr, th); });
    if (!res.ok) {
      RCLCPP_FATAL(this->get_logger(),
        "Autoverificacion de la SECCION 2 (modelo nominal y regresor):\n%s", res.report.c_str());
      throw std::runtime_error("SECCION 2 incompleta o incorrecta");
    }
    RCLCPP_INFO(this->get_logger(),
      "Autoverificacion de la SECCION 2 (modelo nominal y regresor): OK");
  }

  // ── Tick de control a 200 Hz ──────────────────────────────────────────────
  void tick()
  {
    if (!js_.ready()) { return; }

    // Leer estados articulares
    Vec4 q, dq;
    js_.read(q, dq);

    // Captura de la pose inicial + referencia segun fase: transicion quintica
    // con empalme C2 hacia q_d(0), trayectoria de seguimiento, retorno quintico
    // a la pose inicial y pausa en reposo.
    // (Infraestructura, no modificar)
    if (!q0_initialized_) {
      q0_ = q;
      q0_initialized_ = true;
      RCLCPP_INFO(this->get_logger(),
        "Pose inicial capturada: q=[%.3f %.3f %.3f %.3f] rad",
        q0_[0], q0_[1], q0_[2], q0_[3]);
    }

    const Reference goal0 = desiredTrajectory(0.0);
    const bool in_trans    = (t_ < T_TRANS);
    const bool in_tracking = !in_trans && (t_run_ <= 0.0 || t_ < T_TRANS + t_run_);
    Reference ref;
    if (in_trans) {
      ref = omx::quinticTransition(t_, T_TRANS, q0_, goal0);
    } else if (in_tracking) {
      ref = desiredTrajectory(t_ - T_TRANS);
    } else if (t_ < T_TRANS + t_run_ + RETURN_TIME_S) {
      if (!return_logged_) {
        RCLCPP_INFO(this->get_logger(),
          "Iniciando retorno quintico a q_inicial (t=%.1fs, dur=%.1fs)", t_, RETURN_TIME_S);
        return_logged_ = true;
      }
      ref = omx::quinticReturn(t_ - (T_TRANS + t_run_), RETURN_TIME_S,
                               desiredTrajectory(t_run_), q0_);
    } else {
      ref.q = q0_;
      ref.dq.setZero();
      ref.ddq.setZero();
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  [SECCION 4] Ley de control y adaptacion — COMPLETAR
    //  Disponible: q, dq, ref.q, ref.dq, ref.ddq, a_hat_, a_prior_ (12x1),
    //              GAMMA, SIGMA_LEAK (12x1), PHI
    // ─────────────────────────────────────────────────────────────────────────

    //  4.1 Errores articulares:
    const Vec4 e_q  = Vec4::Zero();
    const Vec4 e_dq = Vec4::Zero();

    // 4.2 Superficie deslizante y referencias de Slotine-Li:
    // COMPLETAR
    const Vec4 s_q  = Vec4::Zero();
    const Vec4 dqr  = Vec4::Zero();
    const Vec4 ddqr = Vec4::Zero();

    // Torque nominal y regresor de la SECCION 2, y M (infraestructura, no modificar):
    //   M (4x4), tau_nom (4x1), Y (4x12)
    const Vec4 tanh_dqd = ((ref.dq / FRIC_EPS).array().tanh()).matrix();
    const Eigen::Matrix4d M = model_.massMatrix(q);
    const Vec4 tau_nom = nominalTorque(q, dq, dqr, ddqr, tanh_dqd);
    const MatY Y = regressor(q, dq, dqr, ddqr, tanh_dqd);

    // 4.3 Funcion de conmutacion: rho = rho_vec(s_q, ...)
    // COMPLETAR
    const Vec4 rho = Vec4::Zero();

    // 4.4 Torque:
    const Vec4 tau     = Vec4::Zero();
    const Vec4 tau_sat = tau.cwiseMin(TAU_MAX).cwiseMax(-TAU_MAX);

    // 4.5 Ley de adaptacion (derivada de a_hat, 12x1):
    // COMPLETAR
    const Vec12 a_dot = Vec12::Zero();

    // Suprimir warnings mientras la seccion esta incompleta (eliminar al completarla):
    (void)e_dq; (void)M; (void)tau_nom; (void)Y; (void)rho;
    // ─────────────────────────────────────────────────────────────────────────

    // ── Publicar torques ───────────────────────────────────────────────────
    std_msgs::msg::Float64MultiArray cmd;
    cmd.data.assign(tau_sat.data(), tau_sat.data() + NARM);
    torque_pub_->publish(cmd);

    const char * phase = in_trans    ? "TRANS" :
                         in_tracking ? "TRAY " :
                         (t_ < T_TRANS + t_run_ + RETURN_TIME_S) ? "RET  " : "HOME ";
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "[%s] t=%.2fs  |s|=%.4f  |e|=%.4f rad  alpha=[%.3f %.3f %.3f]  "
      "load=[%.3fkg %.4f %.4f %.4f]  Fv1=%.4f  Fc=[%.4f %.4f %.4f %.4f]",
      phase, t_, s_q.norm(), e_q.norm(),
      a_hat_[0], a_hat_[1], a_hat_[2],
      a_hat_[3], a_hat_[4], a_hat_[5],  a_hat_[6],  a_hat_[7],
      a_hat_[8], a_hat_[9], a_hat_[10], a_hat_[11]);

    // ── Registro CSV (transicion y seguimiento, sin retorno ni pausa)
    if ((in_trans || in_tracking) && csv_.is_open()) {
      csv_ << std::fixed << std::setprecision(6)
           << t_
           << ',' << q[0]        << ',' << q[1]        << ',' << q[2]        << ',' << q[3]
           << ',' << dq[0]       << ',' << dq[1]       << ',' << dq[2]       << ',' << dq[3]
           << ',' << ref.q[0]    << ',' << ref.q[1]    << ',' << ref.q[2]    << ',' << ref.q[3]
           << ',' << ref.dq[0]   << ',' << ref.dq[1]   << ',' << ref.dq[2]   << ',' << ref.dq[3]
           << ',' << s_q[0]      << ',' << s_q[1]      << ',' << s_q[2]      << ',' << s_q[3]
           << ',' << tau_sat[0]  << ',' << tau_sat[1]  << ',' << tau_sat[2]  << ',' << tau_sat[3]
           << ',' << a_hat_[0]   << ',' << a_hat_[1]   << ',' << a_hat_[2]   << ',' << a_hat_[3]
           << ',' << a_hat_[4]   << ',' << a_hat_[5]   << ',' << a_hat_[6]   << ',' << a_hat_[7]
           << ',' << a_hat_[8]   << ',' << a_hat_[9]   << ',' << a_hat_[10]  << ',' << a_hat_[11]
           << '\n';
    }

    // Integracion de a_hat (infraestructura, no modificar): Euler, proyeccion a
    // [A_MIN, A_MAX], solo en la transicion y el seguimiento, y congelada si
    // algun torque satura (anti-windup).
    const bool tau_clipped = (tau.array().abs() > TAU_MAX).any();
    if (adaptive_ && (in_trans || in_tracking) && !tau_clipped) {
      a_hat_ += DT * a_dot;
      a_hat_ = a_hat_.cwiseMax(A_MIN).cwiseMin(A_MAX);
    }

    t_ += DT;

    // Fin: tras el retorno y la pausa en reposo se suelta el torque (el robot
    // queda en la pose de partida, listo para otra ejecucion)
    if (t_run_ > 0.0 && t_ >= T_TRANS + t_run_ + RETURN_TIME_S + HOLD_TIME_S) {
      RCLCPP_INFO(this->get_logger(),
        "Retorno a q_inicial completado (t=%.1f s). Deteniendo control.", t_);
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

  std::array<int, NARM> jid_;
  std::array<Vec10, NARM> pi_nom_;

  Vec4        q0_ {0.0, 0.0, 0.0, 0.0};  // pose medida al arrancar
  bool        q0_initialized_ = false;
  bool        return_logged_  = false;

  double      t_;
  double      t_run_;
  bool        adaptive_;
  std::string mode_str_;
  Vec12       a_prior_;   // a_hat(0) y referencia de la forma de desviacion
  Vec12       a_hat_;     // estimacion actual [alpha1..3, theta_load, Fv1, Fc1..4]
  Vec4        fv_prior_;  // feedforward viscoso FIJO por junta (solo Fv1 se adapta)

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
    rclcpp::spin(std::make_shared<MRACJoint12pSimNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("gz_mrac_joint_12p_node"), "Excepcion: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
