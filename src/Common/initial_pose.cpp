#include "omx/initial_pose.hpp"

#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <vector>

namespace omx
{

namespace
{

constexpr int kPollMs = 100;             // periodo de la espera
constexpr int kStatusEveryCycles = 10;   // una linea de estado por segundo
constexpr int kMaxReadFailures = 20;

struct InitialRange
{
  Vec4 lower, upper;

  bool contains(const Vec4 & q) const
  {
    return (q.array() >= lower.array()).all() && (q.array() <= upper.array()).all();
  }
};

Vec4 declareVec4(rclcpp::Node & node, const std::string & name)
{
  const auto v = node.declare_parameter<std::vector<double>>(name);
  if (v.size() != static_cast<std::size_t>(NUM_JOINTS)) {
    throw std::runtime_error("hw_initial_pose.yaml: " + name + " debe tener 4 valores");
  }
  return Vec4(v[0], v[1], v[2], v[3]);
}

InitialRange declareInitialRange(rclcpp::Node & node)
{
  const Vec4 q_min  = declareVec4(node, "initial_q_min");
  const Vec4 q_max  = declareVec4(node, "initial_q_max");
  const Vec4 margin = declareVec4(node, "initial_q_margin");
  return InitialRange{q_min - margin, q_max + margin};
}

Vec4 readPosition(DynamixelArm & arm)
{
  Vec4 q, dq;
  for (int i = 0; i < kMaxReadFailures; ++i) {
    if (arm.read(q, dq)) return q;
    rclcpp::sleep_for(std::chrono::milliseconds(kPollMs));
  }
  throw std::runtime_error("No se pudo leer la posicion articular del robot");
}

std::string row(const char * label, const Vec4 & v)
{
  char buf[96];
  std::snprintf(buf, sizeof(buf), "  %-16s %8.3f %8.3f %8.3f %8.3f\n", label, v[0], v[1], v[2], v[3]);
  return buf;
}

// Articulaciones fuera de rango y cuanto hay que moverlas, p. ej. "J4 +0.095 rad"
std::string outOfRange(const InitialRange & r, const Vec4 & q)
{
  std::string out;
  for (int i = 0; i < NUM_JOINTS; ++i) {
    const double move = q[i] < r.lower[i] ? r.lower[i] - q[i] :
                        q[i] > r.upper[i] ? r.upper[i] - q[i] : 0.0;
    if (move == 0.0) continue;
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%sJ%d %+.3f rad", out.empty() ? "" : ", ", i + 1, move);
    out += buf;
  }
  return out;
}

void warnOutOfRange(rclcpp::Logger logger, const InitialRange & r, const Vec4 & q)
{
  RCLCPP_WARN(logger,
    "El robot esta fuera de la posicion articular inicial: el control no ha iniciado.\n"
    "  %-16s %8s %8s %8s %8s\n%s%s%s"
    "  Mover: %s\n"
    "Mueva el robot a mano hasta ese rango y presione ENTER para iniciar el control "
    "(Ctrl+C para salir).",
    "[rad]", "J1", "J2", "J3", "J4",
    row("Posicion actual", q).c_str(), row("Minimo", r.lower).c_str(), row("Maximo", r.upper).c_str(),
    outOfRange(r, q).c_str());
}

// Espera hasta kPollMs y retorna true si se presiono ENTER. Sin terminal
// (stdin cerrado) nunca retorna true: el nodo sigue esperando hasta Ctrl+C.
bool enterPressed(bool & stdin_open, rclcpp::Logger logger)
{
  if (!stdin_open) {
    rclcpp::sleep_for(std::chrono::milliseconds(kPollMs));
    return false;
  }
  pollfd pfd{STDIN_FILENO, POLLIN, 0};
  if (::poll(&pfd, 1, kPollMs) <= 0) return false;

  char buf[256];
  const ssize_t n = ::read(STDIN_FILENO, buf, sizeof(buf));
  if (n <= 0) {
    stdin_open = false;
    RCLCPP_ERROR(logger,
      "No hay terminal para leer ENTER: ejecute el nodo con ros2 run. Espera hasta Ctrl+C.");
    return false;
  }
  return true;
}

}  // namespace

bool waitForInitialPose(rclcpp::Node & node, DynamixelArm & arm)
{
  const auto logger = node.get_logger();
  const InitialRange range = declareInitialRange(node);
  const Currents zero = {0, 0, 0, 0};

  Vec4 q = readPosition(arm);
  if (range.contains(q)) {
    RCLCPP_INFO(logger, "q=[%.3f %.3f %.3f %.3f] dentro de la posicion articular inicial: iniciando control.",
      q[0], q[1], q[2], q[3]);
    return true;
  }

  arm.writeCurrents(zero);
  warnOutOfRange(logger, range, q);
  ::tcflush(STDIN_FILENO, TCIFLUSH);   // ignora un ENTER presionado antes del aviso

  bool stdin_open = true;
  int cycle = 0;
  while (rclcpp::ok()) {
    const bool enter = enterPressed(stdin_open, logger);
    arm.writeCurrents(zero);
    q = readPosition(arm);

    if (enter) {
      if (range.contains(q)) {
        RCLCPP_INFO(logger, "q=[%.3f %.3f %.3f %.3f] dentro de la posicion articular inicial: iniciando control.",
          q[0], q[1], q[2], q[3]);
        return true;
      }
      warnOutOfRange(logger, range, q);
    } else if (++cycle % kStatusEveryCycles == 0) {
      if (range.contains(q)) {
        RCLCPP_INFO(logger, "q=[%.3f %.3f %.3f %.3f] dentro del rango: presione ENTER para iniciar.",
          q[0], q[1], q[2], q[3]);
      } else {
        RCLCPP_INFO(logger, "q=[%.3f %.3f %.3f %.3f] mover: %s", q[0], q[1], q[2], q[3],
          outOfRange(range, q).c_str());
      }
    }
  }
  return false;
}

}  // namespace omx
