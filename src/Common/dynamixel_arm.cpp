#include "omx/dynamixel_arm.hpp"

#include <chrono>

namespace omx
{

DynamixelArm::DynamixelArm(rclcpp::Logger logger) : logger_(logger) {}

DynamixelArm::~DynamixelArm() { close(); }

bool DynamixelArm::open(const std::string & port, const Calibration & calib,
                        uint16_t current_limit_register, Mode mode)
{
  port_name_ = port;
  calib_ = calib;
  port_   = dynamixel::PortHandler::getPortHandler(port.c_str());
  packet_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  if (!port_->openPort()) {
    RCLCPP_ERROR(logger_, "No se pudo abrir %s", port.c_str());
    return false;
  }
  if (!port_->setBaudRate(BAUDRATE)) {
    RCLCPP_ERROR(logger_, "No se pudo configurar baudrate");
    port_->closePort();
    return false;
  }

  if (mode == Mode::Current) {
    for (const auto id : DXL_ID)
      write1(id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE_VAL, "pre-disable");
    for (const auto id : DXL_ID) {
      if (!write1(id, ADDR_OPERATING_MODE, CURRENT_CONTROL_MODE, "set mode") ||
          !write2(id, ADDR_CURRENT_LIMIT, current_limit_register, "set limit") ||
          !write1(id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE_VAL, "enable torque")) {
        port_->closePort();
        return false;
      }
      RCLCPP_INFO(logger_, "DXL ID %d listo", static_cast<int>(id));
    }
    wcur_ = std::make_unique<dynamixel::GroupSyncWrite>(
      port_, packet_, ADDR_GOAL_CURRENT, LEN_GOAL_CURRENT);
  } else {
    for (const auto id : DXL_ID) {
      if (!write1(id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE_VAL, "torque disable")) {
        port_->closePort();
        return false;
      }
      RCLCPP_INFO(logger_, "DXL ID %d listo (torque OFF)", static_cast<int>(id));
    }
  }

  reader_ = std::make_unique<StateReader>(port_, packet_, logger_);
  active_ = true;
  RCLCPP_INFO(logger_, "Hardware inicializado en %s", port.c_str());
  return true;
}

bool DynamixelArm::read(Vec4 & q, Vec4 & dq, Currents & cur)
{
  return reader_ && reader_->read(calib_, q, dq, cur);
}

bool DynamixelArm::read(Vec4 & q, Vec4 & dq)
{
  Currents cur{};
  return read(q, dq, cur);
}

bool DynamixelArm::writeCurrents(const Currents & cmd)
{
  if (!wcur_) return false;
  uint8_t p[NUM_JOINTS][2];
  for (int i = 0; i < NUM_JOINTS; ++i) {
    currentToBytes(cmd[i], p[i]);
    wcur_->addParam(DXL_ID[i], p[i]);
  }
  const int r = wcur_->txPacket();
  wcur_->clearParam();
  if (r != COMM_SUCCESS) {
    RCLCPP_ERROR(logger_, "SyncWrite fallo");
    return false;
  }
  return true;
}

void DynamixelArm::close()
{
  if (!active_) return;
  active_ = false;

  if (wcur_) {
    const Currents zero = {0, 0, 0, 0};
    writeCurrents(zero);
    rclcpp::sleep_for(std::chrono::milliseconds(20));
  }
  for (const auto id : DXL_ID)
    write1(id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE_VAL, "shutdown disable");

  if (port_) {
    port_->closePort();
    RCLCPP_INFO(logger_, "Puerto cerrado.");
  }
}

bool DynamixelArm::write1(uint8_t id, uint16_t addr, uint8_t val, const char * lbl)
{
  uint8_t err = 0;
  const int r = packet_->write1ByteTxRx(port_, id, addr, val, &err);
  if (r != COMM_SUCCESS || err != 0) {
    RCLCPP_WARN(logger_, "[ID %d] %s: r=%d err=%d", id, lbl, r, err);
    return false;
  }
  return true;
}

bool DynamixelArm::write2(uint8_t id, uint16_t addr, uint16_t val, const char * lbl)
{
  uint8_t err = 0;
  const int r = packet_->write2ByteTxRx(port_, id, addr, val, &err);
  if (r != COMM_SUCCESS || err != 0) {
    RCLCPP_WARN(logger_, "[ID %d] %s: r=%d err=%d", id, lbl, r, err);
    return false;
  }
  return true;
}

}  // namespace omx
