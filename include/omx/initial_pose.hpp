// Chequeo de la posicion articular inicial de los nodos hw_ de Feedback
// Linearization y SMC. El rango se edita en config/hw_initial_pose.yaml, sin
// recompilar.
//
//   arm_.open(...);
//   if (!omx::waitForInitialPose(*this, arm_)) return;   // Ctrl+C durante la espera
//   // ... iniciar el control
//
// Con el robot dentro del rango, retorna de inmediato. Fuera del rango, muestra la
// posicion actual y el rango, y espera con corriente cero a que el robot se coloque
// a mano y se presione ENTER. Si al presionar ENTER sigue fuera, vuelve a avisar y
// sigue esperando: el nodo no se detiene.
#pragma once

#include <rclcpp/rclcpp.hpp>

#include "omx/dynamixel_arm.hpp"

namespace omx
{

// Retorna true cuando se puede iniciar el control y false si se interrumpio con
// Ctrl+C. Lanza std::runtime_error si no se puede leer el robot.
bool waitForInitialPose(rclcpp::Node & node, DynamixelArm & arm);

}  // namespace omx
