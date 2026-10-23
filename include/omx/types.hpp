// Tipos comunes de los nodos del OpenMANIPULATOR-X: brazo de 4 articulaciones
// (joint1..joint4; la pinza no se controla en estos nodos).
#pragma once

#include <array>
#include <cstdint>

#include <Eigen/Core>

namespace omx
{

constexpr int NUM_JOINTS = 4;

using Vec4     = Eigen::Matrix<double, NUM_JOINTS, 1>;
using Mat4     = Eigen::Matrix<double, NUM_JOINTS, NUM_JOINTS>;
using Currents = std::array<int16_t, NUM_JOINTS>;   // corriente Dynamixel [ticks]

// Referencia articular: posicion [rad], velocidad [rad/s] y aceleracion [rad/s²]
struct Reference { Vec4 q, dq, ddq; };

// Referencia cartesiana del efector y = [x, y, z, phi] y sus dos derivadas
struct CartRef { Vec4 y, ydot, yddot; };

}  // namespace omx
