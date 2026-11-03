/*
 * hw_SMC_cart_node_student.cpp
 * Control por Modo Deslizante en espacio cartesiano — OpenMANIPULATOR-X
 * hardware real vía Dynamixel SDK directo, sin ros2_control.
 * [Archivo base para estudiantes — SMC, implementacion]
 *
 * ──────────────────────────────────────────────────────────────────────────
 * SECCIONES A COMPLETAR:
 *   [1] Trayectoria cartesiana de referencia  →  desiredTrajectory()
 *   [2] Vector Y_START                        →  constante Y_START
 *   [3] Funcion de conmutacion sat(s/phi)     →  rho_scalar()
 *   [4] Ganancias SMC                         →  LAMBDA_Y, K_V, K_S, K_I
 *   [5] Ley SMC cartesiana                    →  compute_control()
 *   [6] Torque                                →  compute_control()
 * El resto (hardware Dynamixel, filtro alpha-beta, transicion quintica,
 * DLS, integracion de z con anti-windup, conversion torque→corriente,
 * guardas de seguridad, CSV) es INFRAESTRUCTURA: no modificar.
 * ──────────────────────────────────────────────────────────────────────────
 *
 * Salida de tarea:  y = [x, y, z, phi]^T
 *   (x,y,z) posición cartesiana del efector final (frame end_effector_link).
 *   phi = q2 + q3 + q4  ángulo de inclinación analítico.
 *
 * Ley de control SMC (ley de alcance exponencial + conmutacion + integral):
 *   e_y  = y - y_d,   edot_y = ydot - ydot_d
 *   s_y  = edot_y + Lambda_y * e_y                       [m/s | rad/s]
 *   v_y  = yddot_d - Lambda_y*edot_y - K_V*s_y - K_S*sat(s_y/phi) - z
 *   qddot = J4^T (J4 J4^T + lambda^2 I)^-1 (v_y - Jdot*qdot)   [DLS]
 *   tau   = M(q)*qddot + Phi(q,qdot)
 *
 * Termino integral de la ley de alcance (parametro integral):
 *   dz/dt = K_I*s_y,   |z_i| <= Z_MAX_i,   z(0) = 0
 *   integral:=false -> z = 0 (SMC clasico)
 *   integral:=true  -> z integra en la transicion y el seguimiento, se congela
 *                      en el retorno y la pausa final, y mientras el torque o
 *                      la corriente saturan. Elimina los sesgos DC que el SMC
 *                      clasico deja por el residual de gravedad/COM del robot.
 *
 * NOTA sobre la friccion: a diferencia de los nodos gz (feedforward del URDF
 * a nivel de torque), en hardware la friccion REAL se compensa en el modelo
 * torque→corriente identificado (Fv, Fc en ticks, con Fc alimentado por la
 * velocidad articular DESEADA qdot_des = J4⁺·ydot_des). NO agregar tau_fric
 * del URDF aqui: seria doble compensacion. El residual de friccion es la
 * incertidumbre acotada que el termino de conmutacion debe dominar.
 *
 * Funcion de conmutacion (elemento a elemento):
 *   rho(s) = sat(s/phi)     phi: capa limite [m/s o rad/s]
 *
 * Trayectoria cartesiana de referencia (serie de Fourier w=1.0 rad/s,
 * identica a gz_SMC_cart_node; FK de la trayectoria articular del SMC):
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
 * indefinidamente). El CSV registra la transicion inicial + el seguimiento (en
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
 *   gain_scale         [double]  1.0    (escala K_V y K_S; K_I con su cuadrado; Lambda fija)
 *   t_run              [double]  20.0   (duracion del SEGUIMIENTO [s];
 *                                        total = T_TRANS + t_run)
 *   test_num           [int]     1      (CSV: hw_smc_cart_sat_<test_num>.csv)
 *   phi                [double]  0.3    (capa limite para sat(s/phi) [m/s o rad/s])
 *   integral           [bool]    false  (activa el termino integral z)
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
 * CSV: data/smc/real/act2/hw_smc_cart_sat_<test_num>.csv
 * Columnas (compatibles con plots_SMC_cart.m, mode='real'):
 *   t, q1..q4, x,y,z,phi, x_des..phi_des, xdot..phidot, xdot_des..phidot_des,
 *   s1..s4, tau1..tau4, cond_J, dq1..dq4, dq1_filt..dq4_filt,
 *   curr_cmd1..4, curr_meas1..4, z1..z4
 * (Sat% se calcula en MATLAB desde tau con el criterio >= 0.99*tau_max. El
 * nombre no distingue integral: usar un test_num distinto para cada ensayo.)
 *
 * Publisher: /hw/joint_states (sensor_msgs/JointState) — monitoreo
 *
 * Ejemplos de ejecucion (primer run SIEMPRE con gain_scale:=0.5 y el brazo
 * pre-posicionado a mano cerca de Y_START):
 *   # SMC clasico:
 *   ros2 run open_manipulator_x_torque_control hw_smc_cart_node_student \
 *     --ros-args -p gain_scale:=0.5 -p test_num:=1 -p t_run:=30.0
 *
 *   # Con termino integral:
 *   ros2 run open_manipulator_x_torque_control hw_smc_cart_node_student \
 *     --ros-args -p integral:=true -p gain_scale:=0.5 -p test_num:=2 -p t_run:=30.0
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

// Tope anti-windup del termino integral |z_i| <= Z_MAX_i [m/s² | rad/s²]
// (INFRAESTRUCTURA, no modificar)
static const Eigen::Vector4d Z_MAX = {15.0, 15.0, 30.0, 320.0};

// Altura mínima del efector sobre la placa metálica de la base [m].
// Se verifica en cada tick para la referencia y para el efector medido.
// La referencia baja hasta z=0.114 → margen de tracking de ~4 cm antes
// de la parada de emergencia.
static constexpr double Z_MIN_FLOOR = 0.075;

static constexpr char EFF_FRAME_NAME[] = "end_effector_link";

using Vec4 = Eigen::Matrix<double, NUM_JOINTS, 1>;

// ═══════════════════════════════════════════════════════════════════════════
//  [SECCION 4] GANANCIAS SMC  [x, y, z, phi] — COMPLETAR
//
//  En HARDWARE la stiction real exige rigidez de tarea varias veces mayor
//  que en Gazebo. Dentro de la capa el SMC equivale a un PD con:
//    kp_eq = Lambda*(K_V + K_S/phi)   kd_eq = Lambda + K_V + K_S/phi
//  Apuntar a la rigidez validada por FL: kp_eq ~ [400, 200, 800, 1500].
//  Con integral:=true se suman kp_eq += K_I y ki_eq = Lambda*K_I.
//
//  Rangos recomendados (con phi = 0.3):
//    LAMBDA_Y : 10 – 40   [1/s]
//    K_V      : 8  – 30
//    K_S      : 1  – 5    [m/s² | rad/s²]
//    K_I      : 2  – 30   [1/s²]           (integral de s; solo con integral:=true)
//  El integral debe ser LENTO: la inercia del rotor no esta en M(q) y reduce la
//  ganancia real de lazo, de modo que un K_I alto oscila aunque elimine el DC.
//  Primer run SIEMPRE con gain_scale:=0.5.
// ═══════════════════════════════════════════════════════════════════════════
static const Eigen::Vector4d LAMBDA_Y = {0.0, 0.0, 0.0, 0.0};   // COMPLETAR
static const Eigen::Vector4d K_V      = {0.0, 0.0, 0.0, 0.0};   // COMPLETAR
static const Eigen::Vector4d K_S      = {0.0, 0.0, 0.0, 0.0};   // COMPLETAR
static const Eigen::Vector4d K_I      = {0.0, 0.0, 0.0, 0.0};   // COMPLETAR
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
//  [SECCION 3] Funcion de conmutacion — COMPLETAR
//
//  Implementar la saturacion (funcion unica en este nodo):
//    sat  : rho(s) = clamp(s / phi, -1, 1)        (phi: capa limite)
// ─────────────────────────────────────────────────────────────────────────────
static double rho_scalar(double s, double phi)
{
  (void)s; (void)phi;

  return 0.0;  // COMPLETAR: sat(s/phi) = clamp(s/phi, -1, 1)
}

// Aplica rho_scalar elemento a elemento sobre el vector s (no modificar)
[[maybe_unused]] static Eigen::Vector4d rho_vec(const Eigen::Vector4d & s, double phi)
{
  Eigen::Vector4d r;
  for (int i = 0; i < NUM_JOINTS; ++i) {
    r[i] = rho_scalar(s[i], phi);
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
    this->declare_parameter<double>     ("gain_scale",  1.0);
    this->declare_parameter<double>     ("t_run",       20.0);
    this->declare_parameter<int>        ("test_num",    1);
    this->declare_parameter<double>     ("phi",         0.3);
    this->declare_parameter<bool>       ("integral",    false);

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
    integral_   = this->get_parameter("integral").as_bool();

    ab_alpha_ = get_parameter("ab_alpha").as_double();
    ab_beta_  = get_parameter("ab_beta").as_double();
    loop_rate_hz_ = std::min(std::max(get_parameter("loop_rate_hz").as_double(), 50.0), 400.0);
    Ts_ = 1.0 / loop_rate_hz_;

    RCLCPP_INFO(get_logger(), "puerto=%s  gain=%.2f  t_run(seguimiento)=%.1fs  test=%d",
      port_name_.c_str(), gain_scale_, t_run_, test_num);
    RCLCPP_INFO(get_logger(),
      "SMC cartesiano hw — rho=sat  phi=%.3f  integral=%s  tau_max=%.2f N·m  T_trans=%.1f s",
      phi_, integral_ ? "true" : "false", hw_.limits.tau_max, T_TRANS);
    RCLCPP_INFO(get_logger(),
      "Lambda_y=[%.1f %.1f %.1f %.1f]  Kv=[%.1f %.1f %.1f %.1f]  Ks=[%.1f %.1f %.1f %.1f]  lambda_DLS=%.3f",
      LAMBDA_Y[0], LAMBDA_Y[1], LAMBDA_Y[2], LAMBDA_Y[3],
      K_V[0],      K_V[1],      K_V[2],      K_V[3],
      K_S[0],      K_S[1],      K_S[2],      K_S[3],
      LAMBDA_DLS);
    if (integral_) {
      RCLCPP_INFO(get_logger(),
        "Ki=[%.1f %.1f %.1f %.1f]  z_max=[%.1f %.1f %.1f %.1f]",
        K_I[0],   K_I[1],   K_I[2],   K_I[3],
        Z_MAX[0], Z_MAX[1], Z_MAX[2], Z_MAX[3]);
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

    // ── CSV ──────────────────────────────────────────────────────────────────
    csv_path_ = std::string(PACKAGE_DATA_DIR) + "/smc/real/act2/hw_smc_cart_sat_"
                + std::to_string(test_num) + ".csv";
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
      "z1,z2,z3,z4",
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
  //  Ley de control SMC cartesiana
  //  gain_scale escala K_V y K_S linealmente y K_I con su cuadrado: el margen
  //  del integral depende de kd*kp, asi que bajar la ganancia no lo reduce.
  //  Lambda define la superficie y no se escala. integrate habilita la
  //  actualizacion del termino integral.
  // ─────────────────────────────────────────────────────────────────────────

  struct SMCOut {
    Vec4            tau;
    Vec4            dq_des;   // velocidad articular deseada (DLS de ydot_des)
    Eigen::Vector4d y_actual, ydot_actual, s_y;
    double          cond_J;
  };

  // Termino integral de la ley de alcance (infraestructura, no modificar):
  // dz/dt = K_I*s, con tope |z_i| <= Z_MAX_i. Vale cero con integral:=false y
  // se congela si el tick anterior saturo torque o corriente.
  void update_integral(const Eigen::Vector4d& s, bool integrate)
  {
    if (integrate && !tau_saturated_prev_) {
      z_ += Ts_ * (K_I * gain_scale_ * gain_scale_).cwiseProduct(s);
      z_ = z_.cwiseMin(Z_MAX).cwiseMax(-Z_MAX);
    }
  }

  SMCOut compute_control(const Vec4& q, const Vec4& dq, const CartRef& ref,
                         bool integrate)
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

    // Pseudo-inversa DLS (infraestructura; factorizacion reutilizada)
    const Eigen::Matrix4d A = J4*J4.transpose() + LAMBDA_DLS_SQ*Eigen::Matrix4d::Identity();
    const Eigen::LDLT<Eigen::Matrix4d> A_ldlt = A.ldlt();

    const Eigen::Vector4d k_v = K_V * gain_scale_;
    const Eigen::Vector4d k_s = K_S * gain_scale_;

    // ═══════════════════════════════════════════════════════════════════════
    //  [SECCION 5] Ley SMC cartesiana — COMPLETAR
    //
    //  Disponible: e_y, edot_y, ref.yddot, LAMBDA_Y, k_v, k_s (ya escaladas),
    //              z_ (termino integral, cero con integral:=false), phi_,
    //              rho_vec(s, phi_)
    //
    //  5.1  s_y = edot_y + Lambda_y * e_y
    //  5.2  rho = rho_vec(s_y, phi_)
    //  5.3  v_y = yddot_d - Lambda_y*edot_y - k_v*s_y - k_s*rho - z_
    // ═══════════════════════════════════════════════════════════════════════

    // 5.1 Superficie deslizante:
    const Eigen::Vector4d s_y = Eigen::Vector4d::Zero();   // COMPLETAR

    // 5.2 Funcion de conmutacion:
    const Eigen::Vector4d rho = Eigen::Vector4d::Zero();   // COMPLETAR

    // Termino integral z_ (infraestructura, no modificar):
    update_integral(s_y, integrate);

    // 5.3 Aceleracion cartesiana virtual:
    const Eigen::Vector4d v_y = Eigen::Vector4d::Zero();   // COMPLETAR

    // Aceleracion articular via DLS (no modificar):
    const Eigen::Vector4d qddot = J4.transpose() * A_ldlt.solve(v_y - jdqd);

    // ═══════════════════════════════════════════════════════════════════════
    //  [SECCION 6] Torque — COMPLETAR
    //
    //  tau_unsat = M4 * qddot + nle4
    //  (en hardware SIN tau_fric del URDF: la friccion real se compensa en
    //   torque_to_current con el modelo identificado del motor)
    // ═══════════════════════════════════════════════════════════════════════
    const Eigen::Vector4d tau_unsat = Eigen::Vector4d::Zero();  // COMPLETAR

    // Suprimir warnings mientras las secciones estan incompletas
    // (eliminar al completarlas):
    (void)M4; (void)nle4; (void)qddot; (void)e_y; (void)edot_y; (void)rho; (void)k_v; (void)k_s;
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

    // 6. Ley de control SMC — usa dq_hat_ para reducir chattering
    const bool integrate = integral_ && (in_trans || in_tracking);
    const SMCOut ctrl = compute_control(q, dq_hat_, ref, integrate);

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
    std::array<bool, NUM_JOINTS> clipped{};
    const auto cur_cmd = omx::torqueToCurrent(hw_, ctrl.tau, dq_hat_, ctrl.dq_des, &clipped);
    tau_saturated_prev_ = tau_saturated_prev_
                        || clipped[0] || clipped[1] || clipped[2] || clipped[3];

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
           << ',' << z_[0] << ',' << z_[1] << ',' << z_[2] << ',' << z_[3]
           << '\n';
    }

    // 12. Log periódico (~1 s)
    if (++log_cnt_ % static_cast<int>(std::lround(loop_rate_hz_)) == 0) {
      if (csv_.is_open()) csv_.flush();
      const char* phase = in_trans    ? "TRANS" :
                          in_tracking ? "TRAY " :
                          (t < T_TRANS + t_run_ + RETURN_TIME_S) ? "RET  " : "HOME ";
      RCLCPP_INFO(get_logger(),
        "[%s] t=%.2fs  |s|=%.4f  |ey|=%.4f m  |z|=%.2f  kJ=%.1f  i=[%d %d %d %d]",
        phase, t, ctrl.s_y.norm(), (ctrl.y_actual - ref.y).norm(), z_.norm(),
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
  double phi_;

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

  bool            integral_{false};
  Eigen::Vector4d z_{Eigen::Vector4d::Zero()};  // termino integral [m/s² | rad/s²]
  bool            tau_saturated_prev_{false};   // anti-windup

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
