// Brazo OpenMANIPULATOR-X (4 servos) en modo corriente o de solo lectura.
//
//   omx::DynamixelArm arm(get_logger());
//   arm.open(port, calib, current_limit_register);   // modo corriente, torque ON
//   arm.read(q, dq, cur);                            // una transaccion por ciclo
//   arm.writeCurrents(cmd);
//   arm.close();                                     // corriente 0, torque OFF
#pragma once

#include <memory>
#include <string>

#include "omx/dynamixel_io.hpp"

namespace omx
{

class DynamixelArm
{
public:
  enum class Mode
  {
    Current,    // modo corriente con torque habilitado
    ReadOnly    // torque deshabilitado: solo se leen posicion, velocidad y corriente
  };

  explicit DynamixelArm(rclcpp::Logger logger);
  ~DynamixelArm();
  DynamixelArm(const DynamixelArm &) = delete;
  DynamixelArm & operator=(const DynamixelArm &) = delete;

  bool open(const std::string & port, const Calibration & calib,
            uint16_t current_limit_register, Mode mode = Mode::Current);
  bool read(Vec4 & q, Vec4 & dq, Currents & cur);
  bool read(Vec4 & q, Vec4 & dq);
  bool writeCurrents(const Currents & cmd);
  void close();   // idempotente

  bool isOpen() const { return active_; }
  const Calibration & calibration() const { return calib_; }

private:
  bool write1(uint8_t id, uint16_t addr, uint8_t val, const char * lbl);
  bool write2(uint8_t id, uint16_t addr, uint16_t val, const char * lbl);

  rclcpp::Logger logger_;
  std::string port_name_;
  Calibration calib_;
  dynamixel::PortHandler * port_{nullptr};
  dynamixel::PacketHandler * packet_{nullptr};
  std::unique_ptr<StateReader> reader_;
  std::unique_ptr<dynamixel::GroupSyncWrite> wcur_;
  bool active_{false};
};

}  // namespace omx
