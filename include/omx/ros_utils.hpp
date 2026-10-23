// Utilidades ROS 2 comunes: opciones de nodo con el YAML del motor, registro CSV
// y lectura de /joint_states en simulacion.
#pragma once

#include <fstream>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "omx/types.hpp"

namespace omx
{

// Opciones del nodo con config/motorXM430W350T_params.yaml auto-cargado. El YAML
// va al final: sus parametros tienen prioridad sobre los -p de la linea de
// comandos; los parametros propios del nodo (no presentes en el YAML) si se
// ajustan con -p.
rclcpp::NodeOptions motorParamsNodeOptions(int argc, char ** argv, const std::string & node_name);

// Crea la carpeta, abre el CSV y escribe el encabezado (sin '\n' final).
bool openCsv(std::ofstream & csv, const std::string & path, const std::string & header,
             rclcpp::Logger logger);

// Ultimo /joint_states recibido, leido por nombre (joint1..joint4), en
// cualquier orden de publicacion.
class JointStateListener
{
public:
  explicit JointStateListener(rclcpp::Node & node, const std::string & topic = "/joint_states");
  bool ready() const { return static_cast<bool>(last_); }
  void read(Vec4 & q, Vec4 & dq) const;   // ceros si aun no llego ningun mensaje
  // true si llego un mensaje nuevo desde la llamada anterior (lo marca como leido)
  bool takeNew() { const bool n = new_msg_; new_msg_ = false; return n; }
  sensor_msgs::msg::JointState::ConstSharedPtr last() const { return last_; }

private:
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  sensor_msgs::msg::JointState::ConstSharedPtr last_;
  bool new_msg_{false};
};

}  // namespace omx
