// Modelo identificado torque → corriente de los XM430-W350-T y parametros de
// hardware (config/motorXM430W350T_params.yaml).
#pragma once

#include <array>

#include <rclcpp/rclcpp.hpp>

#include "omx/dynamixel_io.hpp"

namespace omx
{

// I = alpha·tau + Fv·dq + fc_scale·Fc·tanh(dq_des/epsilon) + I_offset   [ticks]
struct MotorModel
{
  Vec4 alpha, Fv, Fc, I_offset;
  double epsilon{0.05};    // [rad/s] suavizado del tanh de Coulomb
  double fc_scale{0.95};   // fraccion de Fc compensada
};

struct HwLimits
{
  Vec4 joint_lower, joint_upper;     // [rad]
  uint16_t current_limit_register{350};
  Currents current_cmd_limit{{257, 257, 257, 257}};
  int16_t current_measured_peak{313};
  double tau_max{1.2};                // [N·m]
};

struct HwConfig
{
  MotorModel motor;
  Calibration calib;
  HwLimits limits;
};

// Declaran los parametros del YAML del motor. Los limites y la calibracion
// toman como valor por defecto los del YAML; el modelo del motor, los nominales
// (alpha = 208.5, sin friccion ni offset).
MotorModel declareMotorModel(rclcpp::Node & node);
HwLimits declareHwLimits(rclcpp::Node & node);
HwConfig declareHwConfig(rclcpp::Node & node);   // los tres anteriores

// Conversion torque → corriente con el modelo identificado. Fv usa la velocidad
// medida (termino suave) y Fc la velocidad DESEADA: señal sin ruido → sin
// chattering, y empuje de despegue justo cuando la referencia arranca.
// clipped (opcional) indica que articulaciones saturaron en current_cmd_limit.
Currents torqueToCurrent(const HwConfig & hw, const Vec4 & tau, const Vec4 & dq,
                         const Vec4 & dq_des, std::array<bool, NUM_JOINTS> * clipped = nullptr);

}  // namespace omx
