/*
 * hw_fkin_node.cpp
 * Cinematica directa en hardware real — OpenMANIPULATOR-X
 * vía Dynamixel SDK directo, sin ros2_control.
 *
 * Que hace:
 *   1) Se conecta al robot real (omx::DynamixelArm en modo solo lectura).
 *   2) Deja los motores en torque cero: Torque Enable = OFF (bobinados en
 *      circuito abierto). Asi no hay frenado electromagnetico del lazo de
 *      corriente y el brazo se mueve a mano con la minima resistencia posible
 *      (solo queda la friccion mecanica del tren de engranajes, irreducible
 *      por software).
 *   3) Lee las posiciones articulares.
 *   4) Imprime las posiciones articulares actuales [rad].
 *   5) Imprime la posicion y orientacion del efector final (x, y, z, phi)
 *      obtenidos con la cinematica directa analitica (misma fkin que
 *      gz_tvlqr_node.cpp / open_manx_fkin.m).
 *
 * Nota: este nodo NO habilita torque ni entra en modo control de corriente.
 *   El encoder reporta Present Position igual con el torque deshabilitado, y
 *   con Torque Enable = ON aunque la corriente objetivo sea 0 el lazo de
 *   corriente activo introduce un amortiguamiento que se siente como
 *   resistencia extra al mover el brazo. Por eso aqui se deja torque OFF.
 *
 * Parametros ROS 2 (--ros-args -p nombre:=valor):
 *   port_name   [string]  "/dev/ttyUSB0"
 *   Calibracion (joint_zero_tick, encoder_sign): por defecto la del YAML
 *   config/motorXM430W350T_params.yaml (omx::declareCalibration).
 *
 * Publisher: /hw/joint_states (sensor_msgs/JointState) — solo monitoreo
 *
 * ADVERTENCIA: No ejecutar junto a hardware.launch.py ni ningun proceso
 * que acceda a /dev/ttyUSB0 (ros2_control_node, dynamixel_hardware_interface).
 */

#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <Eigen/Dense>

#include "omx/dynamixel_arm.hpp"

using namespace std::chrono_literals;

// ============================================================
// Constantes (servos Dynamixel y unidades: omx/dynamixel_io.hpp)
// ============================================================

static constexpr int NUM_JOINTS = 4;

using Vec4 = Eigen::Matrix<double, NUM_JOINTS, 1>;

// ============================================================
// Cinematica directa analitica (equivalente a open_manx_fkin.m
//   y a la fkin() de gz_tvlqr_node.cpp)
//   Entrada: q = [q1 q2 q3 q4]^T [rad]
//   Salida:  [x, y, z, phi]  con phi = q2 + q3 + q4
// ============================================================

static Eigen::Vector4d fkin(const Eigen::Vector4d & q)
{
  const double x_base = 0.012,  z_base = 0.017 + 0.0595;
  const double x23 = 0.024,  z23 = 0.128;
  const double l34 = 0.124,  l4e = 0.126;

  const double r = x23*std::cos(q[1]) + z23*std::sin(q[1])
                 + l34*std::cos(q[1]+q[2])
                 + l4e*std::cos(q[1]+q[2]+q[3]);
  const double z = z_base
                 + (-x23*std::sin(q[1]) + z23*std::cos(q[1]))
                 - l34*std::sin(q[1]+q[2])
                 - l4e*std::sin(q[1]+q[2]+q[3]);
  return { x_base + r*std::cos(q[0]),
           r*std::sin(q[0]),
           z,
           q[1]+q[2]+q[3] };
}

// ============================================================
// Nodo ROS 2
// ============================================================

class HWFkinNode : public rclcpp::Node
{
public:
  HWFkinNode()
  : Node("hw_fkin_node")
  {
    // ── Parametros ──────────────────────────────────────────────────────────
    this->declare_parameter<std::string>("port_name", "/dev/ttyUSB0");
    port_name_ = this->get_parameter("port_name").as_string();
    calib_     = omx::declareCalibration(*this);

    RCLCPP_INFO(get_logger(), "puerto=%s", port_name_.c_str());

    // ── Publisher de monitoreo ────────────────────────────────────────────────
    js_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/hw/joint_states", 10);

    // ── SDK (torque OFF: bobinados en circuito abierto, sin frenado
    //    electromagnetico; el encoder sigue reportando la posicion) ──────────
    if (!arm_.open(port_name_, calib_, 0, omx::DynamixelArm::Mode::ReadOnly)) {
      RCLCPP_FATAL(get_logger(), "Fallo hardware init. Abortando.");
      throw std::runtime_error("Hardware init failed");
    }

    // ── Timer 10 Hz (lectura + impresion) ─────────────────────────────────────
    timer_ = this->create_wall_timer(100ms, [this]() { tick(); });
    RCLCPP_INFO(get_logger(),
      "Torque deshabilitado (brazo libre). Lectura de cinematica directa a 10 Hz. Ctrl+C para detener.");
  }

  ~HWFkinNode()
  {
    if (timer_) timer_->cancel();
    arm_.close();
    RCLCPP_INFO(get_logger(), "Nodo finalizado.");
  }

private:
  // ─────────────────────────────────────────────────────────────────────────
  //  Callback del timer (10 Hz)
  // ─────────────────────────────────────────────────────────────────────────

  void tick()
  {
    if (!arm_.isOpen()) return;

    // 1. Lectura de posiciones (y velocidades para monitoreo)
    Vec4 q, dq;
    if (!arm_.read(q, dq)) {
      RCLCPP_ERROR(get_logger(), "Lectura de estado fallida.");
      return;
    }

    // 2. Cinematica directa: posicion y orientacion del efector final
    const Eigen::Vector4d y = fkin(q);

    // 3. Imprimir posiciones articulares y pose del efector final
    RCLCPP_INFO(get_logger(),
      "q=[%.4f %.4f %.4f %.4f] rad  |  EE: x=%.4f y=%.4f z=%.4f m  phi=%.4f rad",
      q(0), q(1), q(2), q(3), y(0), y(1), y(2), y(3));

    // 4. Publicar JointState de monitoreo
    sensor_msgs::msg::JointState js;
    js.header.stamp = this->now();
    js.name         = {"joint1", "joint2", "joint3", "joint4"};
    js.position     = {q(0), q(1), q(2), q(3)};
    js.velocity     = {dq(0), dq(1), dq(2), dq(3)};
    js_pub_->publish(js);
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Miembros
  // ─────────────────────────────────────────────────────────────────────────

  std::string port_name_;

  omx::Calibration  calib_;
  omx::DynamixelArm arm_{this->get_logger()};

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

// ============================================================
// main
// ============================================================

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<HWFkinNode>());
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("hw_fkin_node"), "Excepcion: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
