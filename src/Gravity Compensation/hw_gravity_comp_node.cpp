/*
 * hw_gravity_comp_node.cpp
 * Compensacion gravitacional pura — OpenMANIPULATOR-X hardware real.
 * Via Dynamixel SDK directo, sin ros2_control.
 *
 * Modelo dinamico: urdf/open_manipulator_x.urdf
 *   Masas e inercias medidas (balanza + CAD) con origenes en el centro de cada
 *   joint, escaladas a las masas reales. Los torques G(q) son mas precisos que
 *   con el modelo CAD anterior (openmani.urdf).
 *
 * Ley de control:
 *   tau_i = ramp(t) * gravity_scale * gravity_scale_joint[i] * G_i(q)  [N·m]
 *   I     = alpha * tau                                                [ticks]
 *
 *   · ramp(t): rampa quintica 0→1 en soft_start_s. SOSTENER el brazo durante
 *     la rampa; al completarse el nodo avisa ("Puede soltar el robot").
 *   · NO se compensa friccion ni offset del motor (ver torque_to_current):
 *     la friccion residual del reductor es la que "aparca" el brazo en la
 *     pose donde se suelta.
 *
 * Comportamiento: el robot queda sostenido en la pose donde se suelta al
 * iniciar el nodo; al moverlo manualmente a otra pose articular y soltarlo,
 * se mantiene ahi (G(q) se recalcula en cada tick a 100 Hz).
 *
 * Seguridad: parada (torque off) si un joint sale de sus limites articulares
 * (margen 0.05 rad), si |dq| supera dq_max (fuga) o si la corriente medida
 * supera el umbral. Los limites y signos se cargan del YAML de config.
 *
 * Parametros del MOTOR (alpha) y config de hardware (joint_zero_tick, signos,
 * limites articulares): auto-cargados desde config/motorXM430W350T_params.yaml;
 * son FIJOS (no se pueden sobreescribir por CLI — el YAML tiene prioridad).
 *
 * Parametros propios del nodo (ros2 run ... --ros-args -p nombre:=valor):
 *   port_name           [string]   "/dev/ttyUSB0"
 *   gravity_scale       [double]   1.0    (escala global)
 *   gravity_scale_joint [double[]] [1.0, 0.85, 1.0, 1.0]  escala por junta.
 *                        J2=0.85: el URDF sobreestima G2 en ~10-20% (residuo
 *                        dinamico de hw_gc_data_1/2). Ajuste fino por junta:
 *                        si SUBE al soltarla → bajar; si CAE → subir.
 *   soft_start_s        [double]   1.5    (rampa de arranque; 0 = sin rampa)
 *   dq_max              [double]   2.5    (parada de seguridad [rad/s])
 *   duration_s          [double]   0.0    (0 = sin limite de tiempo)
 *   log_id              [int]      1      (CSV: hw_gc_data_<log_id>.csv)
 *
 * Ejemplo de uso (no requiere launch; el YAML se auto-carga):
 *   ros2 run open_manipulator_x_torque_control hw_gravity_comp_node \
 *     --ros-args -p gravity_scale_joint:="[1.0, 0.85, 1.0, 1.0]" -p log_id:=3
 *
 * Publisher:  /hw/joint_states (sensor_msgs/JointState) — monitoreo
 * CSV output: data/gravity_comp/hw_gc_data_<log_id>.csv
 *
 * ADVERTENCIA: No ejecutar junto a otro proceso que acceda a /dev/ttyUSB0
 * (ros2_control_node, dynamixel_hardware_interface, etc.).
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

// Margen tolerado mas alla del limite articular antes de la parada de seguridad
static constexpr double LIMIT_MARGIN_RAD = 0.05;

using Vec4 = Eigen::Matrix<double, NUM_JOINTS, 1>;

// ============================================================
// Nodo ROS 2
// ============================================================

class HWGravityCompNode : public rclcpp::Node
{
public:
  explicit HWGravityCompNode(const rclcpp::NodeOptions & opts = rclcpp::NodeOptions())
  : Node("hw_gravity_comp_node", opts),
    init_logged_(false)
  {
    // ── Parametros propios del nodo (ros2 run ... -p para sobrescribir) ──────
    this->declare_parameter<std::string>("port_name",      "/dev/ttyUSB0");
    this->declare_parameter<double>     ("gravity_scale",   1.0);
    this->declare_parameter<double>     ("soft_start_s",    1.5);
    this->declare_parameter<double>     ("dq_max",          3.0);
    this->declare_parameter<double>     ("duration_s",      0.0);
    this->declare_parameter<int>        ("log_id",          1);

    using dvec = std::vector<double>;
    // Escala de gravedad POR JUNTA (se multiplica con gravity_scale global).
    // J2=0.85: el G2 del URDF sobreestima la gravedad real ~10-20% (residuo
    // dinamico M·ddq+C·dq vs corriente medida en hw_gc_data_1/2); las demas
    // juntas coinciden con el modelo dentro del margen de friccion.
    this->declare_parameter<dvec>("gravity_scale_joint", dvec{1.0, 0.85, 1.0, 1.0});

    // ── Modelo del motor y config de hardware (motorXM430W350T_params.yaml) ──
    // Del modelo del motor solo se usa alpha (I = alpha·tau). Los terminos
    // Fv/Fc/I_offset del modelo identificado NO se aplican en este nodo, a
    // proposito (ver torque_to_current).
    hw_ = omx::declareHwConfig(*this);

    port_name_     = this->get_parameter("port_name").as_string();
    gravity_scale_ = this->get_parameter("gravity_scale").as_double();
    soft_start_s_  = this->get_parameter("soft_start_s").as_double();
    dq_max_        = this->get_parameter("dq_max").as_double();
    duration_s_    = this->get_parameter("duration_s").as_double();
    const int log_id = this->get_parameter("log_id").as_int();

    auto load_vec4 = [this](const std::string & name) {
      const auto v = this->get_parameter(name).as_double_array();
      Vec4 out; for (int i = 0; i < NUM_JOINTS; ++i) out(i) = v[i]; return out;
    };
    gscale_joint_ = load_vec4("gravity_scale_joint");

    RCLCPP_INFO(get_logger(),
      "puerto=%s  scale=%.2f  scale_joint=[%.2f %.2f %.2f %.2f]  rampa=%.1fs  "
      "dq_max=%.1f rad/s  dur=%.1fs  id=%d",
      port_name_.c_str(), gravity_scale_,
      gscale_joint_(0), gscale_joint_(1), gscale_joint_(2), gscale_joint_(3),
      soft_start_s_, dq_max_, duration_s_, log_id);

    // ── Pinocchio ────────────────────────────────────────────────────────────
    const std::string urdf = std::string(PACKAGE_URDF_DIR) + "/open_manipulator_x.urdf";
    RCLCPP_INFO(get_logger(), "Cargando modelo dinamico: %s", urdf.c_str());
    try {
      model_.load(urdf);
    } catch (const std::exception & e) {
      RCLCPP_FATAL(get_logger(), "Pinocchio URDF: %s", e.what());
      throw;
    }
    RCLCPP_INFO(get_logger(), "Pinocchio: nv=%d  masa_total=%.3f kg",
      model_.nv(), model_.totalMass());

    // ── CSV ──────────────────────────────────────────────────────────────────
    const std::string csv_dir = std::string(PACKAGE_DATA_DIR) + "/gravity_comp";
    csv_path_ = csv_dir + "/hw_gc_data_" + std::to_string(log_id) + ".csv";
    omx::openCsv(csv_, csv_path_,
      "t,"
      "q1,q2,q3,q4,"
      "dq1,dq2,dq3,dq4,"
      "tau_g1,tau_g2,tau_g3,tau_g4,"
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

    // ── Timer 100 Hz ─────────────────────────────────────────────────────────
    start_time_ = std::chrono::high_resolution_clock::now();
    timer_ = this->create_wall_timer(10ms, [this]() { tick(); });
    RCLCPP_INFO(get_logger(),
      "Compensacion gravitacional activa a 100 Hz. Ctrl+C para detener.");
  }

  ~HWGravityCompNode()
  {
    if (timer_) timer_->cancel();
    arm_.close();
    if (csv_.is_open()) { csv_.flush(); csv_.close(); }
    RCLCPP_INFO(get_logger(), "Nodo finalizado.");
  }

private:
  // ─────────────────────────────────────────────────────────────────────────
  //  Calculo de torque gravitacional: tau = gravity_scale * G(q)
  //  Usa RNEA con velocidad y aceleracion cero: rnea(q, 0, 0) = G(q)
  // ─────────────────────────────────────────────────────────────────────────

  Vec4 compute_gravity(const Vec4 & q)
  {
    const Vec4 g4 = model_.gravity(q);
    return gravity_scale_ * gscale_joint_.cwiseProduct(g4);
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Conversion torque → corriente:  I = alpha·tau
  //
  //  NOTA: los terminos Fv·dq + Fc·tanh(dq/eps) + I_offset del modelo
  //  identificado NO se aplican aqui, a proposito (diagnostico hw_gc_data_1/2,
  //  donde el brazo se fugaba hacia atras-arriba):
  //   · Fc/Fv con velocidad MEDIDA son realimentacion POSITIVA: empujan en la
  //     direccion de cualquier movimiento (1 LSB de velocidad = 0.024 rad/s ya
  //     activa 45% de Fc con eps=0.05) y eliminan el margen de friccion que
  //     aparca el brazo al soltarlo.
  //   · I_offset (hasta ~0.1 N·m) se identifico en lazo cerrado (FL), donde el
  //     feedback lo absorbia; en lazo abierto es un sesgo directo de torque.
  //  La friccion residual del reductor es la que mantiene la pose al soltar:
  //  el brazo queda quieto donde |G_real - G_comandada| < stiction (~0.1 N·m).
  // ─────────────────────────────────────────────────────────────────────────

  omx::Currents torque_to_current(const Vec4 & tau)
  {
    omx::Currents cmd{};
    for (int i = 0; i < NUM_JOINTS; ++i) {
      const double I = hw_.motor.alpha(i) * tau(i);
      cmd[i] = omx::clampCurrent(hw_.calib.current_sign(i) * hw_.calib.encoder_sign(i) * I, hw_.limits.current_cmd_limit[i]);
    }
    return cmd;
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Parada de emergencia
  // ─────────────────────────────────────────────────────────────────────────

  void emergency_stop(const std::string & reason)
  {
    RCLCPP_ERROR(get_logger(), "PARADA DE EMERGENCIA: %s", reason.c_str());
    timer_->cancel();
    arm_.close();
    if (csv_.is_open()) { csv_.flush(); csv_.close(); }
    rclcpp::shutdown();
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Callback del timer (100 Hz)
  // ─────────────────────────────────────────────────────────────────────────

  void tick()
  {
    if (!arm_.isOpen()) return;

    const auto tp = std::chrono::high_resolution_clock::now();
    const double t = std::chrono::duration<double>(tp - start_time_).count();

    // Limite de duracion
    if (duration_s_ > 0.0 && t >= duration_s_) {
      RCLCPP_INFO(get_logger(), "Duracion completada (%.1f s).", duration_s_);
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

    // 2. Guardas de seguridad: limites articulares y velocidad de fuga
    for (int i = 0; i < NUM_JOINTS; ++i) {
      if (q(i) < hw_.limits.joint_lower(i) - LIMIT_MARGIN_RAD ||
          q(i) > hw_.limits.joint_upper(i) + LIMIT_MARGIN_RAD) {
        emergency_stop("J" + std::to_string(i + 1) + " fuera de limite articular: q="
          + std::to_string(q(i)) + " rad");
        return;
      }
      if (std::abs(dq(i)) > dq_max_) {
        emergency_stop("Velocidad excesiva (posible fuga) en J" + std::to_string(i + 1)
          + ": dq=" + std::to_string(dq(i)) + " rad/s");
        return;
      }
    }

    // 3. Log de posicion inicial (primer tick solamente)
    if (!init_logged_) {
      init_logged_ = true;
      RCLCPP_INFO(get_logger(),
        "Posicion inicial capturada: q=[%.3f %.3f %.3f %.3f] rad. "
        "SOSTENGA el brazo durante la rampa de arranque (%.1f s).",
        q(0), q(1), q(2), q(3), soft_start_s_);
    }

    // 4. Torque de compensacion con rampa quintica de arranque (evita el
    //    escalon de corriente, cuyo impulso a traves del backlash del
    //    reductor rompe la stiction y dispara la fuga)
    const Vec4 tau_grav = compute_gravity(q);
    double ramp = 1.0;
    if (soft_start_s_ > 0.0 && t < soft_start_s_) {
      ramp = omx::quinticProfile(t / soft_start_s_);
    } else if (!ramp_done_) {
      ramp_done_ = true;
      RCLCPP_INFO(get_logger(),
        "Rampa completada: compensacion al 100%%. Puede soltar el robot.");
    }
    const Vec4 tau_cmd = ramp * tau_grav;

    // 5. Convertir torque a corriente y enviar a motores
    const auto cur_cmd = torque_to_current(tau_cmd);
    if (!arm_.writeCurrents(cur_cmd)) {
      emergency_stop("SyncWrite fallido");
      return;
    }

    // 6. Verificar corriente medida (seguridad)
    for (int i = 0; i < NUM_JOINTS; ++i) {
      if (std::abs(cur_meas[i]) > hw_.limits.current_measured_peak) {
        emergency_stop("Corriente medida insegura en J"
          + std::to_string(i + 1)
          + ": " + std::to_string(cur_meas[i]) + " ticks");
        return;
      }
    }

    // 7. Publicar JointState de monitoreo
    {
      sensor_msgs::msg::JointState js;
      js.header.stamp = this->now();
      js.name         = {"joint1", "joint2", "joint3", "joint4"};
      js.position     = {q(0),  q(1),  q(2),  q(3)};
      js.velocity     = {dq(0), dq(1), dq(2), dq(3)};
      js.effort = {
        static_cast<double>(cur_meas[0]) / hw_.motor.alpha(0),
        static_cast<double>(cur_meas[1]) / hw_.motor.alpha(1),
        static_cast<double>(cur_meas[2]) / hw_.motor.alpha(2),
        static_cast<double>(cur_meas[3]) / hw_.motor.alpha(3)
      };
      js_pub_->publish(js);
    }

    // 8. CSV
    if (csv_.is_open()) {
      csv_ << std::fixed << std::setprecision(6) << t
           << ',' << q(0)       << ',' << q(1)       << ',' << q(2)       << ',' << q(3)
           << ',' << dq(0)      << ',' << dq(1)      << ',' << dq(2)      << ',' << dq(3)
           << ',' << tau_cmd(0)  << ',' << tau_cmd(1)  << ',' << tau_cmd(2)  << ',' << tau_cmd(3)
           << ',' << cur_cmd[0] << ',' << cur_cmd[1]  << ',' << cur_cmd[2] << ',' << cur_cmd[3]
           << ',' << cur_meas[0]<< ',' << cur_meas[1] << ',' << cur_meas[2]<< ',' << cur_meas[3]
           << '\n';
    }

    // 9. Log periodico en consola (cada 100 ticks = 1 s)
    if (++log_cnt_ % 100 == 0) {
      if (csv_.is_open()) csv_.flush();
      RCLCPP_INFO(get_logger(),
        "t=%.1fs  q=[%.3f %.3f %.3f %.3f]  "
        "tau_g=[%.3f %.3f %.3f %.3f] N.m  "
        "i_cmd=[%d %d %d %d] ticks",
        t,
        q(0), q(1), q(2), q(3),
        tau_cmd(0), tau_cmd(1), tau_cmd(2), tau_cmd(3),
        cur_cmd[0], cur_cmd[1], cur_cmd[2], cur_cmd[3]);
    }
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Miembros
  // ─────────────────────────────────────────────────────────────────────────

  omx::RobotModel model_;

  std::string port_name_;
  double gravity_scale_;
  double soft_start_s_;
  double dq_max_;
  double duration_s_;
  Vec4   gscale_joint_;
  // Modelo del motor, calibracion y limites (config/motorXM430W350T_params.yaml)
  omx::HwConfig hw_;

  omx::DynamixelArm arm_{this->get_logger()};

  bool init_logged_;
  bool ramp_done_{false};
  std::chrono::high_resolution_clock::time_point start_time_;
  int log_cnt_{0};

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::ofstream csv_;
  std::string   csv_path_;
};

// ============================================================
// main
// ============================================================

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // Auto-carga config/motorXM430W350T_params.yaml; los -p de la línea de comandos
  // (gravity_scale, gravity_scale_joint, soft_start_s, dq_max, duration_s, log_id,
  // port_name) lo sobrescriben → no requiere launch.
  const rclcpp::NodeOptions opts = omx::motorParamsNodeOptions(argc, argv, "hw_gravity_comp_node");

  try {
    rclcpp::spin(std::make_shared<HWGravityCompNode>(opts));
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("hw_gravity_comp_node"),
      "Excepcion: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
