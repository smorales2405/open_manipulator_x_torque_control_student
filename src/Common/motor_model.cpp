#include "omx/motor_model.hpp"

#include <stdexcept>
#include <vector>

namespace omx
{

namespace
{

Vec4 vec4(const std::vector<double> & v, const std::string & name)
{
  if (v.size() != static_cast<std::size_t>(NUM_JOINTS)) {
    throw std::runtime_error("El parametro '" + name + "' debe tener 4 elementos");
  }
  return Vec4(v[0], v[1], v[2], v[3]);
}

}  // namespace

MotorModel declareMotorModel(rclcpp::Node & node)
{
  using dvec = std::vector<double>;
  MotorModel m;
  m.alpha    = vec4(node.declare_parameter<dvec>("motor_alpha",    dvec{208.5, 208.5, 208.5, 208.5}), "motor_alpha");
  m.Fv       = vec4(node.declare_parameter<dvec>("motor_Fv",       dvec{0.0, 0.0, 0.0, 0.0}), "motor_Fv");
  m.Fc       = vec4(node.declare_parameter<dvec>("motor_Fc",       dvec{0.0, 0.0, 0.0, 0.0}), "motor_Fc");
  m.I_offset = vec4(node.declare_parameter<dvec>("motor_I_offset", dvec{0.0, 0.0, 0.0, 0.0}), "motor_I_offset");
  m.epsilon  = node.declare_parameter<double>("motor_epsilon_friction", 0.05);
  m.fc_scale = node.declare_parameter<double>("friction_fc_scale", 0.95);
  return m;
}

HwLimits declareHwLimits(rclcpp::Node & node)
{
  using dvec = std::vector<double>;
  using ivec = std::vector<int64_t>;
  HwLimits l;
  l.joint_lower = vec4(node.declare_parameter<dvec>(
    "joint_lower", motorYamlDoubles("joint_lower", dvec{-2.356194, -1.919862, -1.919862, -1.8})), "joint_lower");
  l.joint_upper = vec4(node.declare_parameter<dvec>(
    "joint_upper", motorYamlDoubles("joint_upper", dvec{+2.356194, +1.745329, +1.570796, +2.1})), "joint_upper");
  l.current_limit_register = static_cast<uint16_t>(node.declare_parameter<int64_t>(
    "current_limit_register", motorYamlInteger("current_limit_register", 350)));
  const auto cl = node.declare_parameter<ivec>(
    "current_cmd_limit", motorYamlIntegers("current_cmd_limit", ivec{257, 257, 257, 257}));
  if (cl.size() != static_cast<std::size_t>(NUM_JOINTS)) {
    throw std::runtime_error("El parametro 'current_cmd_limit' debe tener 4 elementos");
  }
  for (int i = 0; i < NUM_JOINTS; ++i) l.current_cmd_limit[i] = static_cast<int16_t>(cl[i]);
  l.current_measured_peak = static_cast<int16_t>(node.declare_parameter<int64_t>(
    "current_measured_peak", motorYamlInteger("current_measured_peak", 313)));
  l.tau_max = node.declare_parameter<double>("tau_max", 1.2);
  return l;
}

HwConfig declareHwConfig(rclcpp::Node & node)
{
  HwConfig hw;
  hw.motor  = declareMotorModel(node);
  hw.calib  = declareCalibration(node);
  hw.limits = declareHwLimits(node);
  return hw;
}

Currents torqueToCurrent(const HwConfig & hw, const Vec4 & tau, const Vec4 & dq,
                         const Vec4 & dq_des, std::array<bool, NUM_JOINTS> * clipped)
{
  const MotorModel & m = hw.motor;
  Currents cmd{};
  for (int i = 0; i < NUM_JOINTS; ++i) {
    const double I_model = m.alpha(i) * tau(i)
                         + m.Fv(i)    * dq(i)
                         + m.fc_scale * m.Fc(i) * std::tanh(dq_des(i) / m.epsilon)
                         + m.I_offset(i);
    const double I_cmd = hw.calib.current_sign(i) * hw.calib.encoder_sign(i) * I_model;
    if (clipped) {
      (*clipped)[i] = std::abs(I_cmd) > static_cast<double>(hw.limits.current_cmd_limit[i]);
    }
    cmd[i] = clampCurrent(I_cmd, hw.limits.current_cmd_limit[i]);
  }
  return cmd;
}

}  // namespace omx
