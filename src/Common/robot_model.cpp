// Implementacion de omx::RobotModel sobre Pinocchio (unico archivo que lo incluye).
#include "omx/robot_model.hpp"

#include <stdexcept>

#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/regressor.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace omx
{

struct RobotModel::Impl
{
  pinocchio::Model model;
  pinocchio::Data data;
  pinocchio::FrameIndex ee_frame{0};
  bool has_ee{false};

  // Estado del URDF completo: brazo en las primeras 4 coordenadas, pinza en cero
  Eigen::VectorXd full(const Vec4 & x) const
  {
    Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);
    v.head<NUM_JOINTS>() = x;
    return v;
  }

  void requireEndEffector() const
  {
    if (!has_ee) {
      throw std::runtime_error("RobotModel: no se definio el frame del efector");
    }
  }
};

RobotModel::RobotModel() : impl_(std::make_unique<Impl>()) {}
RobotModel::~RobotModel() = default;

void RobotModel::load(const std::string & urdf_path, const std::string & ee_frame)
{
  pinocchio::urdf::buildModel(urdf_path, impl_->model);
  impl_->data = pinocchio::Data(impl_->model);
  impl_->has_ee = false;
  if (!ee_frame.empty()) {
    if (!impl_->model.existFrame(ee_frame)) {
      throw std::runtime_error("Frame no encontrado en el URDF: " + ee_frame);
    }
    impl_->ee_frame = impl_->model.getFrameId(ee_frame);
    impl_->has_ee = true;
  }
}

int RobotModel::nv() const { return impl_->model.nv; }

double RobotModel::totalMass() const
{
  double m = 0.0;
  for (std::size_t i = 1; i < impl_->model.inertias.size(); ++i) m += impl_->model.inertias[i].mass();
  return m;
}

Mat4 RobotModel::massMatrix(const Vec4 & q)
{
  auto & m = *impl_;
  pinocchio::crba(m.model, m.data, m.full(q));
  // crba llena solo el triangulo superior
  m.data.M.triangularView<Eigen::StrictlyLower>() =
    m.data.M.triangularView<Eigen::StrictlyUpper>().transpose();
  return m.data.M.topLeftCorner<NUM_JOINTS, NUM_JOINTS>();
}

Vec4 RobotModel::nonlinearEffects(const Vec4 & q, const Vec4 & dq)
{
  auto & m = *impl_;
  pinocchio::nonLinearEffects(m.model, m.data, m.full(q), m.full(dq));
  return m.data.nle.head<NUM_JOINTS>();
}

Vec4 RobotModel::gravity(const Vec4 & q)
{
  auto & m = *impl_;
  return pinocchio::computeGeneralizedGravity(m.model, m.data, m.full(q)).head<NUM_JOINTS>();
}

Vec4 RobotModel::inverseDynamics(const Vec4 & q, const Vec4 & dq, const Vec4 & ddq)
{
  auto & m = *impl_;
  return pinocchio::rnea(m.model, m.data, m.full(q), m.full(dq), m.full(ddq)).head<NUM_JOINTS>();
}

Eigen::Vector3d RobotModel::eePosition(const Vec4 & q)
{
  auto & m = *impl_;
  m.requireEndEffector();
  pinocchio::forwardKinematics(m.model, m.data, m.full(q));
  pinocchio::updateFramePlacement(m.model, m.data, m.ee_frame);
  return m.data.oMf[m.ee_frame].translation();
}

RobotModel::EndEffectorKinematics RobotModel::eeKinematics(const Vec4 & q, const Vec4 & dq)
{
  auto & m = *impl_;
  m.requireEndEffector();
  const Eigen::VectorXd q_full = m.full(q);
  const Eigen::VectorXd dq_full = m.full(dq);

  EndEffectorKinematics k;
  Eigen::MatrixXd J6 = Eigen::MatrixXd::Zero(6, m.model.nv);
  pinocchio::computeFrameJacobian(m.model, m.data, q_full, m.ee_frame,
                                  pinocchio::LOCAL_WORLD_ALIGNED, J6);
  k.p = m.data.oMf[m.ee_frame].translation();
  k.J = J6.topLeftCorner<3, NUM_JOINTS>();

  pinocchio::forwardKinematics(m.model, m.data, q_full, dq_full, Eigen::VectorXd::Zero(m.model.nv));
  k.dJdq = pinocchio::getFrameClassicalAcceleration(m.model, m.data, m.ee_frame,
                                                    pinocchio::LOCAL_WORLD_ALIGNED).linear();
  return k;
}

Vec4 RobotModel::urdfDamping() const { return impl_->model.damping.head<NUM_JOINTS>(); }
Vec4 RobotModel::urdfFriction() const { return impl_->model.friction.head<NUM_JOINTS>(); }

Vec4 RobotModel::slotineLiTorque(const Vec4 & q, const Vec4 & dq, const Vec4 & dqr, const Vec4 & ddqr)
{
  auto & m = *impl_;
  const Eigen::VectorXd qf = m.full(q), dqf = m.full(dq), dqrf = m.full(dqr), ddqrf = m.full(ddqr);
  const Eigen::VectorXd zero  = Eigen::VectorXd::Zero(m.model.nv);
  const Eigen::VectorXd g     = pinocchio::rnea(m.model, m.data, qf, zero,        zero);
  const Eigen::VectorXd Mddqr = pinocchio::rnea(m.model, m.data, qf, zero,        ddqrf);  // M·ddq_r + g
  const Eigen::VectorXd r_sum = pinocchio::rnea(m.model, m.data, qf, dqf + dqrf,  zero);
  const Eigen::VectorXd r_dq  = pinocchio::rnea(m.model, m.data, qf, dqf,         zero);
  const Eigen::VectorXd r_dqr = pinocchio::rnea(m.model, m.data, qf, dqrf,        zero);
  const Eigen::VectorXd Cqr   = 0.5 * (r_sum - r_dq - r_dqr + g);                         // C(q,dq)·dq_r
  return (Mddqr + Cqr).head<NUM_JOINTS>();
}

Eigen::MatrixXd RobotModel::slotineLiRegressor(const Vec4 & q, const Vec4 & dq, const Vec4 & dqr,
                                               const Vec4 & ddqr)
{
  auto & m = *impl_;
  const Eigen::VectorXd qf = m.full(q), dqf = m.full(dq), dqrf = m.full(dqr), ddqrf = m.full(ddqr);
  const Eigen::VectorXd zero = Eigen::VectorXd::Zero(m.model.nv);
  // Cada llamada sobrescribe data.jointTorqueRegressor: se copia cada resultado
  const Eigen::MatrixXd R_g     = pinocchio::computeJointTorqueRegressor(m.model, m.data, qf, zero,        zero);
  const Eigen::MatrixXd R_Mddqr = pinocchio::computeJointTorqueRegressor(m.model, m.data, qf, zero,        ddqrf);
  const Eigen::MatrixXd R_sum   = pinocchio::computeJointTorqueRegressor(m.model, m.data, qf, dqf + dqrf,  zero);
  const Eigen::MatrixXd R_dq    = pinocchio::computeJointTorqueRegressor(m.model, m.data, qf, dqf,         zero);
  const Eigen::MatrixXd R_dqr   = pinocchio::computeJointTorqueRegressor(m.model, m.data, qf, dqrf,        zero);
  return R_Mddqr + 0.5 * (R_sum - R_dq - R_dqr + R_g);
}

int RobotModel::jointId(const std::string & joint_name) const
{
  if (!impl_->model.existJointName(joint_name)) {
    throw std::runtime_error("Articulacion no encontrada en el URDF: " + joint_name);
  }
  return static_cast<int>(impl_->model.getJointId(joint_name));
}

RobotModel::Vec10 RobotModel::bodyDynamicParameters(int joint_id) const
{
  return impl_->model.inertias[static_cast<std::size_t>(joint_id)].toDynamicParameters();
}

}  // namespace omx
