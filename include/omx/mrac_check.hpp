// Autoverificacion de las plantillas MRAC de 12 parametros: compara el torque
// nominal y el regresor 4x12 escritos en el nodo con la dinamica de Pinocchio,
// en estados de prueba fijos y bloque por bloque. Los nodos la llaman al
// arrancar, antes de enviar torque.
//
// Orden de a_hat: [alpha1..3 | dm, dmcx, dmcy, dmcz | Fv1 | Fc1..4]
#pragma once

#include <array>
#include <functional>
#include <string>

#include <Eigen/Core>

#include "omx/robot_model.hpp"
#include "omx/types.hpp"

namespace omx
{

using MatY12 = Eigen::Matrix<double, NUM_JOINTS, 12>;

// Forma del torque nominal:
//   SlotineLi : M(q)·ddq_r + C(q,dq)·dq_r + g(q)   (nodo de Gazebo)
//   Rnea      : M(q)·ddq_r + n(q,dq)               (nodo de hardware)
// En ambos casos se suma fv_prior.*dq + fc_prior.*tanh(dq_d/eps).
enum class MracNominal { SlotineLi, Rnea };

// Argumentos: (q, dq, dq_r, ddq_r, tanh(dq_d/eps))
using MracTorqueFn = std::function<Vec4(const Vec4 &, const Vec4 &, const Vec4 &,
                                        const Vec4 &, const Vec4 &)>;
using MracRegressorFn = std::function<MatY12(const Vec4 &, const Vec4 &, const Vec4 &,
                                             const Vec4 &, const Vec4 &)>;

struct MracCheckResult
{
  bool ok;
  std::string report;   // una linea por bloque (OK / sin completar / ERROR)
};

// jid: ids de joint1..joint4 (RobotModel::jointId). fv_prior y fc_prior: friccion
// nominal incluida en el torque nominal (cero en hardware).
MracCheckResult checkMrac12p(RobotModel & model, const std::array<int, NUM_JOINTS> & jid,
                             MracNominal form, const Vec4 & fv_prior, const Vec4 & fc_prior,
                             const MracTorqueFn & tau_nom, const MracRegressorFn & regressor);

}  // namespace omx
