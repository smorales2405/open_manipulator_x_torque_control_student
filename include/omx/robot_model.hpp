// Modelo dinamico del OpenMANIPULATOR-X calculado con Pinocchio a partir del URDF.
//
// Es la unica parte del paquete que incluye Pinocchio (src/Common/robot_model.cpp):
// los nodos trabajan solo con Eigen, y compilar cada nodo es mucho mas liviano.
// Todas las funciones reciben el estado de las 4 articulaciones del brazo; la pinza
// del URDF se completa con ceros.
#pragma once

#include <memory>
#include <string>

#include <Eigen/Core>

#include "omx/types.hpp"

namespace omx
{

class RobotModel
{
public:
  using Vec10 = Eigen::Matrix<double, 10, 1>;

  // Cinematica del efector en el marco LOCAL_WORLD_ALIGNED (ejes del mundo)
  struct EndEffectorKinematics
  {
    Eigen::Vector3d p;                    // posicion [m]
    Eigen::Matrix<double, 3, NUM_JOINTS> J;  // jacobiano lineal
    Eigen::Vector3d dJdq;                 // termino de bias J̇·q̇ (aceleracion clasica con q̈ = 0)
  };

  RobotModel();
  ~RobotModel();
  RobotModel(const RobotModel &) = delete;
  RobotModel & operator=(const RobotModel &) = delete;

  // Carga el URDF; ee_frame es el frame del efector (vacio = sin cinematica del efector).
  // Lanza std::runtime_error si el URDF o el frame no existen.
  void load(const std::string & urdf_path, const std::string & ee_frame = "end_effector_link");

  int nv() const;   // grados de libertad del URDF completo (brazo + pinza)
  double totalMass() const;   // suma de las masas de todos los cuerpos [kg]

  Mat4 massMatrix(const Vec4 & q);                                         // M(q)
  Vec4 nonlinearEffects(const Vec4 & q, const Vec4 & dq);                  // n(q,dq) = C(q,dq)·dq + g(q)
  Vec4 gravity(const Vec4 & q);                                            // g(q)
  Vec4 inverseDynamics(const Vec4 & q, const Vec4 & dq, const Vec4 & ddq); // M·ddq + C·dq + g (RNEA)

  Eigen::Vector3d eePosition(const Vec4 & q);
  EndEffectorKinematics eeKinematics(const Vec4 & q, const Vec4 & dq);

  // Friccion articular declarada en el URDF (<dynamics damping/friction>)
  Vec4 urdfDamping() const;
  Vec4 urdfFriction() const;

  // ── Control adaptativo (Slotine-Li) ──────────────────────────────────────
  // tau = M(q)·ddq_r + C(q,dq)·dq_r + g(q) con los parametros nominales del URDF
  Vec4 slotineLiTorque(const Vec4 & q, const Vec4 & dq, const Vec4 & dqr, const Vec4 & ddqr);
  // Regresor completo Y(q,dq,dq_r,ddq_r) (nv x 10·njoints): tau = Y·pi, con
  // las columnas 10·(jid−1) .. 10·(jid−1)+9 del cuerpo movido por la articulacion jid
  Eigen::MatrixXd slotineLiRegressor(const Vec4 & q, const Vec4 & dq, const Vec4 & dqr, const Vec4 & ddqr);
  int jointId(const std::string & joint_name) const;
  Vec10 bodyDynamicParameters(int joint_id) const;   // orden de Inertia::toDynamicParameters()

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace omx
