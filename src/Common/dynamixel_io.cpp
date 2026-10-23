#include "omx/dynamixel_io.hpp"

#include <mutex>
#include <stdexcept>
#include <unordered_map>

#include <rclcpp/parameter_map.hpp>

#ifndef OMX_CONFIG_DIR
#define OMX_CONFIG_DIR "."
#endif

namespace omx
{

namespace
{

// Parametros de config/motorXM430W350T_params.yaml, leidos una sola vez
const std::unordered_map<std::string, rclcpp::Parameter> & motorYaml()
{
  static std::unordered_map<std::string, rclcpp::Parameter> params;
  static std::once_flag once;
  std::call_once(once, [] {
    try {
      for (const auto & [node, list] : rclcpp::parameter_map_from_yaml_file(motorYamlPath())) {
        (void)node;
        for (const auto & p : list) params.emplace(p.get_name(), p);
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN(rclcpp::get_logger("omx"), "No se pudo leer %s (%s): se usan valores nominales",
                  motorYamlPath().c_str(), e.what());
    }
  });
  return params;
}

template <class T>
std::array<T, NUM_JOINTS> toArray4(const std::vector<T> & v, const std::string & name)
{
  if (v.size() != static_cast<std::size_t>(NUM_JOINTS)) {
    throw std::runtime_error("El parametro '" + name + "' debe tener 4 elementos");
  }
  return {v[0], v[1], v[2], v[3]};
}

}  // namespace

std::string motorYamlPath() { return std::string(OMX_CONFIG_DIR) + "/motorXM430W350T_params.yaml"; }

std::vector<double> motorYamlDoubles(const std::string & name, const std::vector<double> & fallback)
{
  const auto & m = motorYaml();
  const auto it = m.find(name);
  return it == m.end() ? fallback : it->second.as_double_array();
}

std::vector<int64_t> motorYamlIntegers(const std::string & name, const std::vector<int64_t> & fallback)
{
  const auto & m = motorYaml();
  const auto it = m.find(name);
  return it == m.end() ? fallback : it->second.as_integer_array();
}

int64_t motorYamlInteger(const std::string & name, int64_t fallback)
{
  const auto & m = motorYaml();
  const auto it = m.find(name);
  return it == m.end() ? fallback : it->second.as_int();
}

Calibration declareCalibration(rclcpp::Node & node)
{
  using dvec = std::vector<double>;
  using ivec = std::vector<int64_t>;
  const auto zt = node.declare_parameter<ivec>(
    "joint_zero_tick", motorYamlIntegers("joint_zero_tick", ivec{2048, 2048, 2048, 2048}));
  const auto es = node.declare_parameter<dvec>(
    "encoder_sign", motorYamlDoubles("encoder_sign", dvec{+1.0, +1.0, +1.0, +1.0}));
  const auto cs = node.declare_parameter<dvec>(
    "current_sign", motorYamlDoubles("current_sign", dvec{+1.0, +1.0, +1.0, +1.0}));

  Calibration c;
  const auto z = toArray4(zt, "joint_zero_tick");
  for (int i = 0; i < NUM_JOINTS; ++i) c.joint_zero_tick[i] = static_cast<int32_t>(z[i]);
  const auto e = toArray4(es, "encoder_sign");
  const auto s = toArray4(cs, "current_sign");
  c.encoder_sign << e[0], e[1], e[2], e[3];
  c.current_sign << s[0], s[1], s[2], s[3];
  return c;
}

StateReader::StateReader(dynamixel::PortHandler * port, dynamixel::PacketHandler * packet,
                         rclcpp::Logger logger)
: grp_(port, packet, ADDR_STATE_BLOCK, LEN_STATE_BLOCK), logger_(logger)
{
  for (const auto id : DXL_ID) grp_.addParam(id);
}

bool StateReader::read(const Calibration & calib, Vec4 & q, Vec4 & dq, Currents & cur)
{
  if (grp_.txRxPacket() != COMM_SUCCESS) {
    RCLCPP_ERROR(logger_, "SyncRead fallo");
    return false;
  }
  for (int i = 0; i < NUM_JOINTS; ++i) {
    const uint8_t id = DXL_ID[i];
    if (!grp_.isAvailable(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION) ||
        !grp_.isAvailable(id, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY) ||
        !grp_.isAvailable(id, ADDR_PRESENT_CURRENT,  LEN_PRESENT_CURRENT)) {
      RCLCPP_ERROR(logger_, "[ID %d] dato no disponible", id);
      return false;
    }
    const int32_t rp = toSigned32(static_cast<uint32_t>(
      grp_.getData(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)));
    const int32_t rv = toSigned32(static_cast<uint32_t>(
      grp_.getData(id, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY)));
    const int16_t rc = toSigned16(static_cast<uint32_t>(
      grp_.getData(id, ADDR_PRESENT_CURRENT,  LEN_PRESENT_CURRENT)));

    q(i)   = calib.encoder_sign(i) * static_cast<double>(wrappedTickDiff(rp, calib.joint_zero_tick[i])) * POS_UNIT_RAD;
    dq(i)  = calib.encoder_sign(i) * static_cast<double>(rv) * VEL_UNIT_RAD_S;
    cur[i] = rc;
  }
  return true;
}

}  // namespace omx
