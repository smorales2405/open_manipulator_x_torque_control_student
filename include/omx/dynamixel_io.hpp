// Comunicacion de bajo nivel con los servos XM430-W350-T del OpenMANIPULATOR-X
// (Protocolo 2.0): tabla de control, unidades, conversiones, calibracion del
// ensamblado y lectura del estado en una sola transaccion de bus.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "omx/types.hpp"

namespace omx
{

// ── Servos y bus ─────────────────────────────────────────────────────────────
inline constexpr std::array<uint8_t, NUM_JOINTS> DXL_ID = {11, 12, 13, 14};
inline constexpr int    BAUDRATE         = 1000000;
inline constexpr double PROTOCOL_VERSION = 2.0;

// ── Tabla de control ─────────────────────────────────────────────────────────
inline constexpr uint16_t ADDR_OPERATING_MODE   = 11;
inline constexpr uint16_t ADDR_CURRENT_LIMIT    = 38;
inline constexpr uint16_t ADDR_TORQUE_ENABLE    = 64;
inline constexpr uint16_t ADDR_GOAL_CURRENT     = 102;
inline constexpr uint16_t ADDR_GOAL_VELOCITY    = 104;
inline constexpr uint16_t ADDR_PROFILE_ACC      = 108;
inline constexpr uint16_t ADDR_PROFILE_VEL      = 112;
inline constexpr uint16_t ADDR_GOAL_POSITION    = 116;
inline constexpr uint16_t ADDR_PRESENT_CURRENT  = 126;
inline constexpr uint16_t ADDR_PRESENT_VELOCITY = 128;
inline constexpr uint16_t ADDR_PRESENT_POSITION = 132;

inline constexpr uint16_t LEN_GOAL_CURRENT     = 2;
inline constexpr uint16_t LEN_GOAL_VELOCITY    = 4;
inline constexpr uint16_t LEN_GOAL_POSITION    = 4;
inline constexpr uint16_t LEN_PRESENT_CURRENT  = 2;
inline constexpr uint16_t LEN_PRESENT_VELOCITY = 4;
inline constexpr uint16_t LEN_PRESENT_POSITION = 4;

// Bloque contiguo corriente(126,2) + velocidad(128,4) + posicion(132,4):
// una sola GroupSyncRead por ciclo en vez de tres → latencia de bus ≈ 1/3
inline constexpr uint16_t ADDR_STATE_BLOCK = ADDR_PRESENT_CURRENT;
inline constexpr uint16_t LEN_STATE_BLOCK  = 10;

inline constexpr uint8_t CURRENT_CONTROL_MODE  = 0;
inline constexpr uint8_t VELOCITY_CONTROL_MODE = 1;
inline constexpr uint8_t POSITION_CONTROL_MODE = 3;
inline constexpr uint8_t TORQUE_ENABLE_VAL     = 1;
inline constexpr uint8_t TORQUE_DISABLE_VAL    = 0;

// ── Unidades ─────────────────────────────────────────────────────────────────
inline constexpr double PI                      = 3.14159265358979323846;
inline constexpr double POS_UNIT_RAD            = 2.0 * PI / 4096.0;
inline constexpr double VEL_UNIT_RAD_S          = 0.229 * 2.0 * PI / 60.0;
inline constexpr double CURRENT_UNIT_A          = 0.00269;
inline constexpr double TORQUE_CONSTANT_NM_A    = 1.654;
inline constexpr double TORQUE_PER_CURRENT_TICK = TORQUE_CONSTANT_NM_A * CURRENT_UNIT_A;

// ── Conversiones de bytes ────────────────────────────────────────────────────
inline int32_t toSigned32(uint32_t v)
{
  if (v > 0x7FFFFFFFu) return -static_cast<int32_t>(0xFFFFFFFFu - v + 1u);
  return static_cast<int32_t>(v);
}

inline int16_t toSigned16(uint32_t v)
{
  const uint16_t w = static_cast<uint16_t>(v & 0xFFFFu);
  if (w > 0x7FFFu) return -static_cast<int16_t>(0xFFFFu - w + 1u);
  return static_cast<int16_t>(w);
}

// Diferencia de ticks respecto del cero, envuelta a [-2048, 2048]
inline int32_t wrappedTickDiff(int32_t raw, int32_t zero)
{
  int32_t d = raw - zero;
  while (d >  2048) d -= 4096;
  while (d < -2048) d += 4096;
  return d;
}

inline void currentToBytes(int16_t cur, uint8_t p[2])
{
  const uint16_t w = static_cast<uint16_t>(cur);
  p[0] = DXL_LOBYTE(w);
  p[1] = DXL_HIBYTE(w);
}

inline int16_t clampCurrent(double x, int16_t lim)
{
  return static_cast<int16_t>(std::lround(
    std::min(std::max(x, -static_cast<double>(lim)), static_cast<double>(lim))));
}

// ── Calibracion del ensamblado ───────────────────────────────────────────────
// joint_zero_tick : tick del encoder en q = 0
// encoder_sign    : sentido del encoder respecto del URDF (+1 / -1)
// current_sign    : sentido de la corriente respecto del torque (+1 / -1)
struct Calibration
{
  std::array<int32_t, NUM_JOINTS> joint_zero_tick{{2048, 2048, 2048, 2048}};
  Vec4 encoder_sign = Vec4::Ones();
  Vec4 current_sign = Vec4::Ones();
};

// Declara joint_zero_tick, encoder_sign y current_sign tomando como valor por
// defecto los de config/motorXM430W350T_params.yaml, aunque el nodo no cargue
// ese YAML (asi todos los nodos usan la misma calibracion).
Calibration declareCalibration(rclcpp::Node & node);

// ── Lectura del estado ───────────────────────────────────────────────────────
// Lee corriente, velocidad y posicion de los 4 servos en una sola GroupSyncRead
// y las convierte a unidades articulares con la calibracion.
class StateReader
{
public:
  StateReader(dynamixel::PortHandler * port, dynamixel::PacketHandler * packet, rclcpp::Logger logger);
  bool read(const Calibration & calib, Vec4 & q, Vec4 & dq, Currents & cur);

private:
  dynamixel::GroupSyncRead grp_;
  rclcpp::Logger logger_;
};

// Valor de un parametro de config/motorXM430W350T_params.yaml (o fallback si no existe)
std::vector<double> motorYamlDoubles(const std::string & name, const std::vector<double> & fallback);
std::vector<int64_t> motorYamlIntegers(const std::string & name, const std::vector<int64_t> & fallback);
int64_t motorYamlInteger(const std::string & name, int64_t fallback);
std::string motorYamlPath();

}  // namespace omx
