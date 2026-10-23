/*
 * hw_SMC_cart_node_student.cpp
 * Control por Modo Deslizante en espacio cartesiano — OpenMANIPULATOR-X
 * hardware real vía Dynamixel SDK directo, sin ros2_control.
 * [Archivo base para estudiantes — Lab 6, implementacion]
 *
 * Soporta DOS leyes de control, seleccionables con el parametro booleano
 * use_integral (default: false):
 *   · use_integral:=false → SMC clasico (superficie en velocidad)
 *   · use_integral:=true  → ISMC, SMC con termino INTEGRAL (superficie en
 *     posicion con estado xi = ∫e dt) — elimina los sesgos DC que el SMC
 *     clasico deja por el residual del modelo de gravedad/COM del robot.
 *
 * ──────────────────────────────────────────────────────────────────────────
 * SECCIONES A COMPLETAR:
 *   [1] Trayectoria cartesiana de referencia  →  desiredTrajectory()
 *   [2] Vector Y_START                        →  constante Y_START
 *   [3] Funciones de conmutacion              →  rho_scalar()
 *   [4] Ganancias SMC clasico                 →  LAMBDA_Y, K_V, K_S
 *   [5] Ganancias ISMC                        →  LAMBDA_I, K_P, K_D, K_sw
 *   [6] Ley SMC clasica                       →  compute_control() (rama false)
 *   [7] Ley ISMC                              →  compute_control() (rama true)
 *   [8] Torque                                →  compute_control() (comun)
 * El resto (hardware Dynamixel, filtro alpha-beta, transicion quintica,
 * DLS, integracion de xi con anti-windup, conversion torque→corriente,
 * guardas de seguridad, CSV) es INFRAESTRUCTURA: no modificar.
 * ──────────────────────────────────────────────────────────────────────────
 *
 * Salida de tarea:  y = [x, y, z, phi]^T
 *   (x,y,z) posición cartesiana del efector final (frame end_effector_link).
 *   phi = q2 + q3 + q4  ángulo de inclinación analítico.
 *
 * Ley SMC clasica (use_integral:=false):
 *   e_y  = y - y_d,   edot_y = ydot - ydot_d
 *   s_y  = edot_y + Lambda_y * e_y                       [m/s | rad/s]
 *   v_y  = yddot_d - Lambda_y*edot_y - K_v*s_y - K_s*rho(s_y)
 *   qddot = J4^T (J4 J4^T + lambda^2 I)^-1 (v_y - Jdot*qdot)   [DLS]
 *   tau   = M(q)*qddot + Phi(q,qdot)
 *
 * Ley ISMC (use_integral:=true), formulacion de hw_SMCI_cart_node:
 *   xi   = ∫ e dt  (lo integra la INFRAESTRUCTURA, con anti-windup)
 *   s    = e + Lambda_I * xi                             [m | rad]
 *   sdot = edot + Lambda_I * e
 *   v_s  = yddot_d - Lambda_I*edot - Jdot*qdot - K_D*sdot - K_P*s - K_sw*rho(s)
 *   qddot = J4^T (J4 J4^T + lambda^2 I)^-1 v_s           [DLS]
 *   tau   = M(q)*qddot + Phi(q,qdot)
 *
 * NOTA sobre la friccion: a diferencia de los nodos gz (feedforward del URDF
 * a nivel de torque), en hardware la friccion REAL se compensa en el modelo
 * torque→corriente identificado (Fv, Fc en ticks, con Fc alimentado por la
 * velocidad articular DESEADA qdot_des = J4⁺·ydot_des). NO agregar tau_fric
 * del URDF aqui: seria doble compensacion. El residual de friccion es la
 * incertidumbre acotada que el termino de conmutacion debe dominar.
 *
 * Funciones de conmutacion (elemento a elemento):
 *   "sign" -> rho(s) = sign(s)
 *   "sat"  -> rho(s) = sat(s/phi)     phi: capa limite [m/s o rad/s]
 *
 * Trayectoria cartesiana de referencia (serie de Fourier w=1.0 rad/s,
 * identica a gz_SMC_cart_node; FK de la trayectoria articular del Lab 6):
 *   x_d   = 0.172 + 0.032*sin(t') + 0.027*cos(2t') + 0.003*sin(3t')
 *   y_d   = 0.015 + 0.136*sin(t') - 0.014*cos(2t') + 0.003*sin(3t') - 0.001*cos(4t')
 *   z_d   = 0.128 - 0.008*sin(t') + 0.006*cos(2t')      [z ∈ 0.114..0.142 m]
 *   phi_d = 0.685 + 0.250*sin(t')
 * Transicion inicial [0, T_TRANS): quintica generalizada desde la pose
 * medida hasta Y_START = FK(q_d(0)) con empalme C2 (velocidad y aceleracion
 * de la serie en t'=0).
 *
 * Fases temporales completas (ver constantes T_TRANS/RETURN_TIME_S/HOLD_TIME_S):
 *   [0, T_TRANS)                                  transicion quintica a Y_START
 *   [T_TRANS, T_TRANS+t_run)                      seguimiento periodico
 *   [T_TRANS+t_run, T_TRANS+t_run+RETURN_TIME_S)  retorno quintico a y_inicial
 *   [...+RETURN_TIME_S, ...+HOLD_TIME_S)          pausa en reposo (asienta)
 *   >= ...+RETURN_TIME_S+HOLD_TIME_S              corte de corriente y fin
 * El retorno evita que el brazo caiga por gravedad al recibir torque cero de
 * golpe en medio de la trayectoria (t_run<=0 desactiva el retorno: corre
 * indefinidamente). El estado integral xi_ (si use_integral:=true) NO se
 * actualiza durante la transicion, el retorno ni la pausa (solo durante el
 * seguimiento). El CSV registra la transicion inicial + el seguimiento (en
 * hardware las fallas suelen ocurrir en la transicion); plots_SMC_cart.m en
 * modo 'real' descarta t < T_TRANS para que las metricas sean solo de
 * seguimiento. El retorno y la pausa final NO se guardan en el CSV.
 *
 * PROCEDIMIENTO DE ARRANQUE: posicionar el brazo a mano cerca de Y_START
 * (codo arriba, efector a ~0.20 m y ~0.13 m de altura) antes de lanzar el
 * nodo. Arrancar desde la pose de reposo caida (brazo extendido-bajo) exige
 * levantar el brazo extendido contra gravedad+stiction durante la
 * transicion y fue la causa de la divergencia del test1 hw.
 *
 * Guardas de altura mínima (placa metálica de la base):
 *   La referencia y el efector medido nunca pueden bajar de Z_MIN_FLOOR
 *   (0.075 m; la referencia baja hasta 0.114 → margen de tracking de ~4 cm).
 *   Durante la transición solo se exige no descender por debajo de la pose
 *   inicial (el brazo puede arrancar plegado, con z < Z_MIN_FLOOR).
 *
 * Guarda de singularidad:
 *   Parada de emergencia si cond(J4) > COND_J_MAX (150). En el test1 hw la
 *   configuracion derivo a una singularidad (kJ 23 → 18869) y la DLS
 *   repartio comandos grandes en direcciones mal condicionadas hasta llevar
 *   J1 a su limite; esta guarda corta esa divergencia ~1 s antes.
 *
 * Parametros ROS 2 (--ros-args -p nombre:=valor):
 *   port_name          [string]  "/dev/ttyUSB0"
 *   use_integral       [bool]    false  (false = SMC clasico | true = ISMC)
 *   gain_scale         [double]  1.0    (escala lineal de las ganancias de la
 *                                        ley activa; Lambdas fijas)
 *   t_run              [double]  20.0   (duracion del SEGUIMIENTO [s];
 *                                        total = T_TRANS + t_run)
 *   test_num           [int]     1      (CSV: hw_smc_cart_... o hw_smci_cart_...
 *                                        segun use_integral)
 *   rho_func           [string]  "sat"  (funcion de conmutacion: "sign" | "sat")
 *   phi                [double]  0.3    (capa limite para sat(s/phi); OJO con
 *                                        las unidades: SMC clasico s en [m/s |
 *                                        rad/s] → usar ~0.3; ISMC s en [m |
 *                                        rad] → usar ~0.02, -p phi:=0.02)
 *   ab_alpha           [double]  0.2    (filtro α-β: ganancia de posicion)
 *   ab_beta            [double]  0.02   (filtro α-β: ganancia de velocidad)
 *   friction_fc_scale  [double]  0.95   (fraccion de Fc compensada)
 *   loop_rate_hz       [double]  200.0  (frecuencia del lazo [Hz], acotada [50,400])
 *
 * Parametros del modelo identificado (auto-cargados desde
 * config/motorXM430W350T_params.yaml): motor_alpha, motor_Fv, motor_Fc,
 * motor_I_offset, motor_epsilon_friction, joint_zero_tick, encoder_sign,
 * current_sign, joint_lower/upper, current_*_limit, tau_max.
 *
 * CSV: data/lab6/real/act2/hw_smc_cart_<rho_func>_<test_num>.csv   (SMC)
 *      data/lab6/real/act2/hw_smci_cart_<rho_func>_<test_num>.csv  (ISMC)
 * Columnas (compatibles con plots_SMC_cart.m, mode='real',
 * controller='smc'|'smci' segun use_integral):
 *   t, q1..q4, x,y,z,phi, x_des..phi_des, xdot..phidot, xdot_des..phidot_des,
 *   s1..s4, tau1..tau4, cond_J, dq1..dq4, dq1_filt..dq4_filt,
 *   curr_cmd1..4, curr_meas1..4, xi1..xi4
 * (Sat% se calcula en MATLAB desde tau con el criterio >= 0.99*tau_max.)
 *
 * Publisher: /hw/joint_states (sensor_msgs/JointState) — monitoreo
 *
 * Ejemplos de ejecucion (primer run SIEMPRE con gain_scale:=0.5 y el brazo
 * pre-posicionado a mano cerca de Y_START):
 *   # SMC clasico:
 *   ros2 run open_manipulator_x_torque_control hw_smc_cart_node_student \
 *     --ros-args -p gain_scale:=0.5 -p rho_func:=sat -p test_num:=1 -p t_run:=30.0
 *
 *   # ISMC (termino integral habilitado; phi en unidades de POSICION):
 *   ros2 run open_manipulator_x_torque_control hw_smc_cart_node_student \
 *     --ros-args -p use_integral:=true -p phi:=0.02 -p rho_func:=sat \
 *     -p test_num:=2 -p t_run:=30.0
 *
 * ADVERTENCIA: No ejecutar junto a hardware.launch.py ni ningun proceso
 * que acceda a /dev/ttyUSB0 (ros2_control_node, dynamixel_hardware_interface).
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
#include <Eigen/LU>
#include <Eigen/SVD>

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

// Amortiguamiento DLS para pseudo-inversa (evita divergencia en singularidades)
static constexpr double LAMBDA_DLS    = 0.01;
static constexpr double LAMBDA_DLS_SQ = LAMBDA_DLS * LAMBDA_DLS;

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

// Transicion mas larga que en sim (5 s vs 3 s): levantar el brazo real
// contra gravedad+stiction con demandas suaves. (No modificar)
static constexpr double T_TRANS = 5.0;   // [s]
// Retorno con la misma duracion que la transicion: bajar el brazo tambien
// enfrenta gravedad+stiction. (No modificar)
static constexpr double RETURN_TIME_S = 5.0;   // [s]
static constexpr double HOLD_TIME_S   = 0.5;   // pausa en reposo antes de cortar corriente

// Umbral de parada por mal condicionamiento del Jacobiano de tarea
// (no modificar)
static constexpr double COND_J_MAX = 150.0;

// ── Anti-windup del estado integral xi (INFRAESTRUCTURA, no modificar) ──────
// Tope POR CANAL |xi_i| <= XI_MAX_i (~2-3x el xi de equilibrio esperado por
// los sesgos del robot; un tope holgado deja que Lambda*xi almacene una
// falsa referencia y desestabilice el brazo en los picos de demanda).
static const Eigen::Vector4d XI_MAX = {0.015, 0.015, 0.015, 0.03};  // [m·s | rad·s]
// Compuerta POR CANAL: xi_i solo integra con |s_i| <= gate_i (en unidades de
// POSICION). x/y/z: 0.02 m (engrana en regimen, se congela en las rafagas de
// saturacion). phi: 0.15 rad (su sesgo tipico ~0.11 debe quedar dentro).
static const Eigen::Vector4d XI_S_GATE = {0.02, 0.02, 0.02, 0.15};  // [m | rad]

// Altura mínima del efector sobre la placa metálica de la base [m].
// Se verifica en cada tick para la referencia y para el efector medido.
// La referencia baja hasta z=0.114 → margen de tracking de ~4 cm antes
// de la parada de emergencia.
static constexpr double Z_MIN_FLOOR = 0.075;

static constexpr char EFF_FRAME_NAME[] = "end_effector_link";

using Vec4 = Eigen::Matrix<double, NUM_JOINTS, 1>;

// ═══════════════════════════════════════════════════════════════════════════
//  [SECCION 4] GANANCIAS SMC CLASICO  [x, y, z, phi] — COMPLETAR
//  (se usan cuando use_integral:=false)
//
//  En HARDWARE la stiction real exige rigidez de tarea varias veces mayor
//  que en Gazebo. Dentro de la capa el SMC equivale a un PD con:
//    kp_eq = Lambda*(K_V + K_S/phi)   kd_eq = Lambda + K_V + K_S/phi
//  Apuntar a la rigidez validada por FL: kp_eq ~ [400, 200, 800, 1500].
//
//  Rangos recomendados (con phi = 0.3):
//    LAMBDA_Y : 10 – 40   [1/s]
//    K_V      : 8  – 30
//    K_S      : 1  – 5    [m/s² | rad/s²]
//  Primer run SIEMPRE con gain_scale:=0.5.
// ═══════════════════════════════════════════════════════════════════════════
static const Eigen::Vector4d LAMBDA_Y = {0.0, 0.0, 0.0, 0.0};   // COMPLETAR
static const Eigen::Vector4d K_V      = {0.0, 0.0, 0.0, 0.0};   // COMPLETAR
static const Eigen::Vector4d K_S      = {0.0, 0.0, 0.0, 0.0};   // COMPLETAR
// ═══════════════════════════════════════════════════════════════════════════

// ═══════════════════════════════════════════════════════════════════════════
//  [SECCION 5] GANANCIAS ISMC  [x, y, z, phi] — COMPLETAR
//  (se usan cuando use_integral:=true)
//
//  Lambda_I: polo de la superficie  s = e + Lambda_I*xi        [1/s]
//  K_P:      ganancia proporcional sobre s en sddot            [1/s²]
//  K_D:      ganancia derivativa  sobre sdot en sddot          [1/s]
//  K_sw:     ganancia de conmutacion rho(s)                    [m/s² | rad/s²]
//
//  Dentro de la capa (rho = s/phi) la dinamica del error es un PID con:
//    kp_eq = Lambda_I*K_D + K_P + K_sw/phi     kd_eq = Lambda_I + K_D
//    ki_eq = (K_P + K_sw/phi)*Lambda_I
//  Apuntar a los mismos kp_eq/kd_eq del SMC clasico (la novedad es ki_eq,
//  que elimina los sesgos DC). Lambda_I ~ sqrt(kp_eq)/4 separa el polo
//  integral de la dinamica PD.
//
//  Rangos recomendados (con phi = 0.02 — ¡s en unidades de POSICION!):
//    LAMBDA_I : 3  – 12   [1/s]
//    K_P      : 50 – 500  [1/s²]
//    K_D      : 20 – 70   [1/s]
//    K_sw     : 1  – 10   [m/s² | rad/s²]
// ═══════════════════════════════════════════════════════════════════════════
static const Eigen::Vector4d LAMBDA_I = {0.0, 0.0, 0.0, 0.0};   // COMPLETAR
static const Eigen::Vector4d K_P      = {0.0, 0.0, 0.0, 0.0};   // COMPLETAR
static const Eigen::Vector4d K_D      = {0.0, 0.0, 0.0, 0.0};   // COMPLETAR
static const Eigen::Vector4d K_sw     = {0.0, 0.0, 0.0, 0.0};   // COMPLETAR
// ═══════════════════════════════════════════════════════════════════════════

// ============================================================
// Trayectoria cartesiana (identica a gz_SMC_cart_node)
// ============================================================

using omx::CartRef;

// ─────────────────────────────────────────────────────────────────────────────
//  [SECCION 1] Trayectoria cartesiana de referencia — COMPLETAR
//
//  Definir la trayectoria como una serie de Fourier (w = 1.0 rad/s, misma
//  trayectoria que en Gazebo):
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
[[maybe_unused]] static Eigen::Vector4d rho_vec(const Eigen::Vector4d & s, RhoFunc func, double phi)
{
  Eigen::Vector4d r;
  for (int i = 0; i < NUM_JOINTS; ++i) {
    r[i] = rho_scalar(s[i], func, phi);
  }
  return r;
}

// ============================================================
// Nodo ROS 2
// ============================================================

class HWSMCCartNode : public rclcpp::Node
{
public:
  explicit HWSMCCartNode(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions())
  : Node("hw_smc_cart_node", opts), y0_initialized_(false)
  {
    // ── Parámetros ──────────────────────────────────────────────────────────
    this->declare_parameter<std::string>("port_name",  "/dev/ttyUSB0");
    this->declare_parameter<bool>       ("use_integral", false);
    this->declare_parameter<double>     ("gain_scale",  1.0);
    this->declare_parameter<double>     ("t_run",       20.0);
    this->declare_parameter<int>        ("test_num",    1);
    this->declare_parameter<std::string>("rho_func",    "sat");
    this->declare_parameter<double>     ("phi",         0.3);

    this->declare_parameter<double>("ab_alpha", 0.2);
    this->declare_parameter<double>("ab_beta",  0.02);
    this->declare_parameter<double>("loop_rate_hz", 200.0);

    // Modelo del motor, calibracion y limites (config/motorXM430W350T_params.yaml)
    hw_ = omx::declareHwConfig(*this);

    port_name_    = this->get_parameter("port_name").as_string();
    use_integral_ = this->get_parameter("use_integral").as_bool();
    gain_scale_   = this->get_parameter("gain_scale").as_double();
    t_run_        = this->get_parameter("t_run").as_double();
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
      "%s cartesiano hw — rho=%s  phi=%.4f  tau_max=%.2f N·m  T_trans=%.1f s",
      use_integral_ ? "ISMC" : "SMC",
      rho_str_.c_str(), phi_, hw_.limits.tau_max, T_TRANS);
    if (use_integral_) {
      RCLCPP_INFO(get_logger(),
        "Lambda_I=[%.1f %.1f %.1f %.1f]  K_P=[%.0f %.0f %.0f %.0f]  "
        "K_D=[%.0f %.0f %.0f %.0f]  K_sw=[%.1f %.1f %.1f %.1f]  lambda_DLS=%.3f",
        LAMBDA_I[0], LAMBDA_I[1], LAMBDA_I[2], LAMBDA_I[3],
        K_P[0],      K_P[1],      K_P[2],      K_P[3],
        K_D[0],      K_D[1],      K_D[2],      K_D[3],
        K_sw[0],     K_sw[1],     K_sw[2],     K_sw[3],
        LAMBDA_DLS);
      RCLCPP_INFO(get_logger(),
        "XI_max=[%.3f %.3f %.3f %.3f]  XI_s_gate=[%.2f %.2f %.2f %.2f]",
        XI_MAX[0], XI_MAX[1], XI_MAX[2], XI_MAX[3],
        XI_S_GATE[0], XI_S_GATE[1], XI_S_GATE[2], XI_S_GATE[3]);
    } else {
      RCLCPP_INFO(get_logger(),
        "Lambda_y=[%.1f %.1f %.1f %.1f]  Kv=[%.1f %.1f %.1f %.1f]  Ks=[%.1f %.1f %.1f %.1f]  lambda_DLS=%.3f",
        LAMBDA_Y[0], LAMBDA_Y[1], LAMBDA_Y[2], LAMBDA_Y[3],
        K_V[0],      K_V[1],      K_V[2],      K_V[3],
        K_S[0],      K_S[1],      K_S[2],      K_S[3],
        LAMBDA_DLS);
    }
    RCLCPP_INFO(get_logger(),
      "Y_start=[%.3f %.3f %.3f %.3f]  Z_min_floor=%.3f m",
      Y_START[0], Y_START[1], Y_START[2], Y_START[3], Z_MIN_FLOOR);
    RCLCPP_INFO(get_logger(),
      "motor α=[%.1f %.1f %.1f %.1f]  Fv=[%.2f %.2f %.2f %.2f]  ε=%.3f",
      hw_.motor.alpha(0), hw_.motor.alpha(1), hw_.motor.alpha(2), hw_.motor.alpha(3),
      hw_.motor.Fv(0), hw_.motor.Fv(1), hw_.motor.Fv(2), hw_.motor.Fv(3), hw_.motor.epsilon);
    RCLCPP_INFO(get_logger(),
      "Filtro α-β: α=%.3f  β=%.4f  |  Fc_scale=%.2f  |  lazo=%.0f Hz",
      ab_alpha_, ab_beta_, hw_.motor.fc_scale, loop_rate_hz_);

    // ── Pinocchio ────────────────────────────────────────────────────────────
    const std::string urdf = std::string(PACKAGE_URDF_DIR) + "/open_manipulator_x.urdf";
    try {
      model_.load(urdf, EFF_FRAME_NAME);   // lanza si falta el URDF o el frame del efector
    } catch (const std::exception& e) {
      RCLCPP_FATAL(get_logger(), "Pinocchio URDF: %s", e.what());
      throw;
    }
    RCLCPP_INFO(get_logger(), "Pinocchio: nv=%d  frame='%s'", model_.nv(), EFF_FRAME_NAME);

    // ── CSV (prefijo segun la ley activa; compatible con plots_SMC_cart.m
    //    controller='smc'|'smci') ────────────────────────────────────────────
    csv_path_ = std::string(PACKAGE_DATA_DIR) + "/lab6/real/act2/"
                + (use_integral_ ? "hw_smci_cart_" : "hw_smc_cart_")
                + rho_str_ + "_" + std::to_string(test_num) + ".csv";
    omx::openCsv(csv_, csv_path_,
      "t,q1,q2,q3,q4,"
      "x,y,z,phi,"
      "x_des,y_des,z_des,phi_des,"
      "xdot,ydot,zdot,phidot,"
      "xdot_des,ydot_des,zdot_des,phidot_des,"
      "s1,s2,s3,s4,"
      "tau1,tau2,tau3,tau4,"
      "cond_J,"
      "dq1,dq2,dq3,dq4,"
      "dq1_filt,dq2_filt,dq3_filt,dq4_filt,"
      "curr_cmd1,curr_cmd2,curr_cmd3,curr_cmd4,"
      "curr_meas1,curr_meas2,curr_meas3,curr_meas4,"
      "xi1,xi2,xi3,xi4",
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
    RCLCPP_INFO(get_logger(), "Control SMC cartesiano activo a %.0f Hz (Ts=%.1f ms). Ctrl+C para detener.",
      loop_rate_hz_, 1e3 * Ts_);
  }

  ~HWSMCCartNode()
  {
    if (timer_) timer_->cancel();
    arm_.close();
    if (csv_.is_open()) { csv_.flush(); csv_.close(); }
    RCLCPP_INFO(get_logger(), "Nodo finalizado.");
  }

private:
  // ─────────────────────────────────────────────────────────────────────────
  //  Conversión torque → corriente
  //  Fv usa la velocidad medida (término suave); Fc usa la velocidad articular
  //  DESEADA (mapeo DLS de ydot_des): señal sin ruido → sin chattering, y
  //  aporta el empuje de despegue cuando la referencia arranca.
  // ─────────────────────────────────────────────────────────────────────────

  // ─────────────────────────────────────────────────────────────────────────
  //  Ley de control cartesiana (SMC clasico o ISMC segun use_integral)
  //  gain_scale escala las ganancias de la ley activa; las Lambdas definen
  //  la superficie y no se escalan. Actualiza el estado integral xi_ (solo
  //  con use_integral:=true).
  // ─────────────────────────────────────────────────────────────────────────

  struct SMCOut {
    Vec4            tau;
    Vec4            dq_des;   // velocidad articular deseada (DLS de ydot_des)
    Eigen::Vector4d y_actual, ydot_actual, s_y;
    double          cond_J;
  };

  SMCOut compute_control(const Vec4& q, const Vec4& dq, const CartRef& ref,
                         bool in_tracking)
  {
    // FK, jacobiano lineal y bias J̇·q̇ del efector (LOCAL_WORLD_ALIGNED) (infraestructura, no modificar)
    const auto ee = model_.eeKinematics(q, dq);

    const Eigen::Vector3d p   = ee.p;
    const double phi_ee       = q[1] + q[2] + q[3];

    // Jacobiano de tarea 4×4
    Eigen::Matrix4d J4;
    J4.topRows<3>() = ee.J;
    J4.row(3) << 0.0, 1.0, 1.0, 1.0;

    const Eigen::Vector4d ydot = J4 * dq;

    // Término de bias  Jdot*qdot  (FK con qddot=0)
    Eigen::Vector4d jdqd;
    jdqd << ee.dJdq, 0.0;

    // Dinámica M(q) y Phi(q,qdot)
    const Eigen::Matrix4d M4   = model_.massMatrix(q);
    const Eigen::Vector4d nle4 = model_.nonlinearEffects(q, dq);

    // Errores cartesianos (infraestructura)
    const Eigen::Vector4d y_actual{p[0], p[1], p[2], phi_ee};
    const Eigen::Vector4d e_y    = y_actual - ref.y;
    const Eigen::Vector4d edot_y = ydot     - ref.ydot;

    // Estado integral xi (INFRAESTRUCTURA, no modificar): solo con
    // use_integral y en fase de seguimiento (in_tracking — ni transicion,
    // ni retorno, ni pausa); compuerta por canal |s_i| <= XI_S_GATE_i,
    // congelado si el tick anterior saturo algun torque, y acotado a
    // |xi_i| <= XI_MAX_i.
    if (use_integral_ && in_tracking && !tau_saturated_prev_) {
      for (int i = 0; i < NUM_JOINTS; ++i) {
        const double s_pre = e_y[i] + LAMBDA_I[i] * xi_[i];
        if (std::abs(s_pre) <= XI_S_GATE[i]) {
          xi_[i] += e_y[i] * Ts_;
          xi_[i] = std::max(-XI_MAX[i], std::min(XI_MAX[i], xi_[i]));
        }
      }
    }

    // Pseudo-inversa DLS (infraestructura; factorizacion reutilizada)
    const Eigen::Matrix4d A = J4*J4.transpose() + LAMBDA_DLS_SQ*Eigen::Matrix4d::Identity();
    const Eigen::LDLT<Eigen::Matrix4d> A_ldlt = A.ldlt();

    Eigen::Vector4d s_y   = Eigen::Vector4d::Zero();
    Eigen::Vector4d qddot = Eigen::Vector4d::Zero();

    if (!use_integral_) {
      // ═════════════════════════════════════════════════════════════════════
      //  [SECCION 6] Ley SMC clasica — COMPLETAR
      //
      //  Disponible: e_y, edot_y, ref.yddot, jdqd, LAMBDA_Y, K_V, K_S,
      //              gain_scale_, rho_vec(s, rho_func_, phi_), A_ldlt, J4
      //
      //  6.1  s_y = edot_y + Lambda_y * e_y            (superficie, ya declarada)
      //  6.2  rho = rho_vec(s_y, rho_func_, phi_)
      //  6.3  v_y = yddot_d - Lambda_y*edot_y - k_v*s_y - k_s*rho
      //       (k_v = K_V*gain_scale_, k_s = K_S*gain_scale_)
      //  6.4  qddot = J4^T * A_ldlt.solve(v_y - jdqd)  (ya dada abajo)
      // ═════════════════════════════════════════════════════════════════════

      // COMPLETAR: s_y (asignar, no redeclarar), rho y v_y
      const Eigen::Vector4d v_y = Eigen::Vector4d::Zero();   // COMPLETAR

      // Aceleracion articular via DLS (no modificar):
      qddot = J4.transpose() * A_ldlt.solve(v_y - jdqd);
      // ═════════════════════════════════════════════════════════════════════
    } else {
      // ═════════════════════════════════════════════════════════════════════
      //  [SECCION 7] Ley ISMC — COMPLETAR
      //
      //  Disponible: e_y, edot_y, ref.yddot, jdqd, xi_ (ya actualizado por
      //              la infraestructura), LAMBDA_I, K_P, K_D, K_sw,
      //              gain_scale_, rho_vec(s, rho_func_, phi_), A_ldlt, J4
      //
      //  7.1  s_y  = e_y + Lambda_I * xi_              (superficie integral)
      //  7.2  sdot_y = edot_y + Lambda_I * e_y
      //  7.3  rho  = rho_vec(s_y, rho_func_, phi_)     (¡phi en unidades de posicion!)
      //  7.4  v_s  = yddot_d - Lambda_I*edot_y - jdqd
      //              - k_d*sdot_y - k_p*s_y - k_sw*rho
      //       (k_p = K_P*gain_scale_, etc.)
      //  7.5  qddot = J4^T * A_ldlt.solve(v_s)         (ya dada abajo)
      // ═════════════════════════════════════════════════════════════════════

      // COMPLETAR: s_y (asignar, no redeclarar), sdot_y, rho y v_s
      const Eigen::Vector4d v_s = Eigen::Vector4d::Zero();   // COMPLETAR

      // Aceleracion articular via DLS (no modificar):
      qddot = J4.transpose() * A_ldlt.solve(v_s);
      // ═════════════════════════════════════════════════════════════════════
    }

    // ═══════════════════════════════════════════════════════════════════════
    //  [SECCION 8] Torque — COMPLETAR (comun a ambas leyes)
    //
    //  tau_unsat = M4 * qddot + nle4
    //  (en hardware SIN tau_fric del URDF: la friccion real se compensa en
    //   torque_to_current con el modelo identificado del motor)
    // ═══════════════════════════════════════════════════════════════════════
    const Eigen::Vector4d tau_unsat = Eigen::Vector4d::Zero();  // COMPLETAR

    // Suprimir warnings mientras las secciones estan incompletas
    // (eliminar al completarlas):
    (void)M4; (void)nle4; (void)qddot; (void)edot_y;
    // ═══════════════════════════════════════════════════════════════════════

    // Saturacion + flag de anti-windup (infraestructura, no modificar)
    const Eigen::Vector4d tau_sat = tau_unsat.cwiseMin(hw_.limits.tau_max).cwiseMax(-hw_.limits.tau_max);
    tau_saturated_prev_ = (tau_unsat.cwiseAbs().maxCoeff() >= hw_.limits.tau_max);

    // Condicionamiento del Jacobiano (monitoreo de singularidades)
    Eigen::JacobiSVD<Eigen::Matrix4d> svd(J4);
    const double sigma_min = svd.singularValues()(NUM_JOINTS - 1);

    SMCOut out;
    out.tau         = tau_sat;
    out.dq_des      = J4.transpose() * A_ldlt.solve(ref.ydot);
    out.y_actual    = y_actual;
    out.ydot_actual = ydot;
    out.s_y         = s_y;
    out.cond_J      = (sigma_min > 1e-10) ? svd.singularValues()(0) / sigma_min : 1e10;
    return out;
  }

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
    const auto tp  = std::chrono::high_resolution_clock::now();
    const double t = std::chrono::duration<double>(tp - start_time_).count();

    const double t_shutdown = T_TRANS + t_run_ + RETURN_TIME_S + HOLD_TIME_S;
    if (t_run_ > 0.0 && t >= t_shutdown) {
      RCLCPP_INFO(get_logger(), "Retorno a y_inicial completado. Deteniendo (t=%.1f s).", t);
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

    // 2. Filtro α-β: estimación conjunta de posición y velocidad
    //   predicción: q_pred = q_hat + Ts·dq_hat,  dq_pred = dq_hat
    //   residuo:    r      = q_meas - q_pred
    //   corrección: q_hat  = q_pred + α·r,        dq_hat = dq_pred + (β/Ts)·r
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

    // 3. Capturar pose cartesiana inicial (primer tick)
    if (!y0_initialized_) {
      const Eigen::Vector3d p0 = model_.eePosition(q);
      const double phi0        = q[1] + q[2] + q[3];
      y0_ = Eigen::Vector4d{p0[0], p0[1], p0[2], phi0};
      y0_initialized_ = true;
      RCLCPP_INFO(get_logger(),
        "y_inicial=[%.3f %.3f %.3f %.3f]  Y_START=[%.3f %.3f %.3f %.3f]",
        y0_[0], y0_[1], y0_[2], y0_[3],
        Y_START[0], Y_START[1], Y_START[2], Y_START[3]);
    }

    // 4. Verificar límites articulares del estado real
    for (int i = 0; i < NUM_JOINTS; ++i) {
      if (q(i) < hw_.limits.joint_lower(i) || q(i) > hw_.limits.joint_upper(i)) {
        emergency_stop("Articulacion " + std::to_string(i+1) + " fuera de limites: "
                       + std::to_string(q(i)) + " rad");
        return;
      }
    }

    // 5. Referencia cartesiana: transicion quintica C2 hacia Y_START (con
    //    velocidad/aceleracion de la serie en t'=0), seguimiento, y retorno
    //    quintico final + pausa en reposo (ver RETURN_TIME_S/HOLD_TIME_S)
    CartRef goal0 = desiredTrajectory(0.0);
    goal0.y = Y_START;
    const bool in_trans    = (t < T_TRANS);
    const bool in_tracking = !in_trans && (t_run_ <= 0.0 || t < T_TRANS + t_run_);
    CartRef ref;
    if (in_trans) {
      ref = omx::quinticTransition(t, T_TRANS, y0_, goal0);
    } else if (in_tracking) {
      ref = desiredTrajectory(t - T_TRANS);
    } else if (t < T_TRANS + t_run_ + RETURN_TIME_S) {
      if (!return_logged_) {
        RCLCPP_INFO(get_logger(), "Iniciando retorno quintico a y_inicial (t=%.1fs, dur=%.1fs)",
          t, RETURN_TIME_S);
        return_logged_ = true;
      }
      ref = omx::quinticReturn(t - (T_TRANS + t_run_), RETURN_TIME_S, desiredTrajectory(t_run_), y0_);
    } else {
      ref.y = y0_;
      ref.ydot.setZero();
      ref.yddot.setZero();
    }

    // 5b. Guarda de altura mínima de la REFERENCIA (placa base metálica).
    //     Cerca de la pose inicial (transición de entrada, retorno final o
    //     pausa en reposo) el brazo puede estar bajo (y0_[2] < Z_MIN_FLOOR):
    //     solo se exige no descender más de 3 cm bajo la pose inicial.
    const bool near_home = in_trans || (t_run_ > 0.0 && t >= T_TRANS + t_run_);
    const double z_floor = near_home
      ? std::min(Z_MIN_FLOOR, y0_[2] - 0.03)
      : Z_MIN_FLOOR;
    if (ref.y[2] < z_floor) {
      emergency_stop("Referencia z bajo altura minima: "
                     + std::to_string(ref.y[2]) + " m (piso " + std::to_string(z_floor) + ")");
      return;
    }

    // 6. Ley de control (SMC clasico o ISMC) — usa dq_hat_ para reducir
    //    chattering; actualiza el estado integral xi_ internamente (solo
    //    en in_tracking)
    const SMCOut ctrl = compute_control(q, dq_hat_, ref, in_tracking);

    // 6b. Guarda de altura mínima del EFECTOR medido (FK de Pinocchio)
    if (ctrl.y_actual[2] < z_floor) {
      emergency_stop("Efector bajo altura minima: z="
                     + std::to_string(ctrl.y_actual[2]) + " m (piso " + std::to_string(z_floor) + ")");
      return;
    }

    // 6c. Guarda de singularidad: cerca de una singularidad la DLS reparte
    //     comandos grandes en direcciones mal condicionadas (test1 hw:
    //     kJ 23 → 18869 y J1 se fue a su limite).
    if (ctrl.cond_J > COND_J_MAX) {
      emergency_stop("Jacobiano mal condicionado: cond(J4)="
                     + std::to_string(ctrl.cond_J) + " (umbral " + std::to_string(COND_J_MAX) + ")");
      return;
    }

    // 7. Conversión torque → corriente (Fc con la velocidad articular deseada)
    const auto cur_cmd = omx::torqueToCurrent(hw_, ctrl.tau, dq_hat_, ctrl.dq_des);

    // 8. Escribir corriente
    if (!arm_.writeCurrents(cur_cmd)) {
      emergency_stop("SyncWrite fallido");
      return;
    }

    // 9. Verificar corriente medida
    for (int i = 0; i < NUM_JOINTS; ++i) {
      if (std::abs(cur_meas[i]) > hw_.limits.current_measured_peak) {
        emergency_stop("Corriente insegura J" + std::to_string(i+1)
                       + ": " + std::to_string(cur_meas[i]) + " ticks");
        return;
      }
    }

    // 10. Publicar JointState de monitoreo
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

    // 11. CSV (incluye la transicion inicial y el seguimiento; excluye el
    //     retorno final a y_inicial y la pausa en reposo. En hardware las
    //     fallas suelen ocurrir en la transicion; plots_SMC_cart.m modo
    //     'real' descarta t < T_TRANS)
    const bool in_return_or_hold = (t_run_ > 0.0 && t >= T_TRANS + t_run_);
    if (csv_.is_open() && !in_return_or_hold) {
      csv_ << std::fixed << std::setprecision(6)
           << t
           << ',' << q(0) << ',' << q(1) << ',' << q(2) << ',' << q(3)
           << ',' << ctrl.y_actual[0] << ',' << ctrl.y_actual[1]
           << ',' << ctrl.y_actual[2] << ',' << ctrl.y_actual[3]
           << ',' << ref.y[0] << ',' << ref.y[1] << ',' << ref.y[2] << ',' << ref.y[3]
           << ',' << ctrl.ydot_actual[0] << ',' << ctrl.ydot_actual[1]
           << ',' << ctrl.ydot_actual[2] << ',' << ctrl.ydot_actual[3]
           << ',' << ref.ydot[0] << ',' << ref.ydot[1]
           << ',' << ref.ydot[2] << ',' << ref.ydot[3]
           << ',' << ctrl.s_y[0] << ',' << ctrl.s_y[1]
           << ',' << ctrl.s_y[2] << ',' << ctrl.s_y[3]
           << ',' << ctrl.tau[0] << ',' << ctrl.tau[1]
           << ',' << ctrl.tau[2] << ',' << ctrl.tau[3]
           << ',' << ctrl.cond_J
           << ',' << dq(0) << ',' << dq(1) << ',' << dq(2) << ',' << dq(3)
           << ',' << dq_hat_(0) << ',' << dq_hat_(1) << ',' << dq_hat_(2) << ',' << dq_hat_(3)
           << ',' << cur_cmd[0] << ',' << cur_cmd[1]
           << ',' << cur_cmd[2] << ',' << cur_cmd[3]
           << ',' << cur_meas[0] << ',' << cur_meas[1]
           << ',' << cur_meas[2] << ',' << cur_meas[3]
           << ',' << xi_[0] << ',' << xi_[1] << ',' << xi_[2] << ',' << xi_[3]
           << '\n';
    }

    // 12. Log periódico (~1 s)
    if (++log_cnt_ % static_cast<int>(std::lround(loop_rate_hz_)) == 0) {
      if (csv_.is_open()) csv_.flush();
      const char* phase = in_trans    ? "TRANS" :
                          in_tracking ? "TRAY " :
                          (t < T_TRANS + t_run_ + RETURN_TIME_S) ? "RET  " : "HOME ";
      RCLCPP_INFO(get_logger(),
        "[%s] t=%.2fs  |s|=%.4f  |ey|=%.4f m  |xi|=%.4f  kJ=%.1f  i=[%d %d %d %d]",
        phase, t, ctrl.s_y.norm(), (ctrl.y_actual - ref.y).norm(), xi_.norm(),
        ctrl.cond_J, cur_cmd[0], cur_cmd[1], cur_cmd[2], cur_cmd[3]);
    }

    // 13. Detección de overrun: si el ciclo (bus + control) no cabe en Ts,
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

  bool y0_initialized_;
  Eigen::Vector4d y0_;
  bool return_logged_{false};

  bool            use_integral_{false};
  Eigen::Vector4d xi_{Eigen::Vector4d::Zero()};  // estado integral ∫e dt
  bool            tau_saturated_prev_{false};    // anti-windup condicional

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

  const rclcpp::NodeOptions opts = omx::motorParamsNodeOptions(argc, argv, "hw_smc_cart_node");

  try {
    rclcpp::spin(std::make_shared<HWSMCCartNode>(opts));
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("hw_smc_cart_node"), "Excepcion: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
