// Retorno final en espacio articular para los nodos de control cartesiano (IO).
//
// Al cumplirse t_run el nodo deja de seguir su referencia cartesiana y lleva el
// brazo en reposo a la pose articular inicial: quintica articular desde el
// estado medido al empezar el retorno y PD articular con compensacion de
// gravedad (g del URDF):
//   tau = g(q) + Kp·(q_ref − q) + Kd·(dq_ref − dq)
//
// Por que articular: en un retorno cartesiano la muneca (J4) no tiene
// referencia propia y compensa los errores de J2/J3 para sostener
// phi = q2 + q3 + q4. Con la pose inicial cerca del limite de J4 (brazo plegado
// en reposo, J4 ≈ −1.70 rad con limite −1.80) el sobrepaso al llegar disparaba
// la parada por limites articulares en el robot real.
//
// Ganancias por plataforma (forHardware / forSimulation): con un PD de
// ganancias constantes, el lazo discreto de 200 Hz es estable solo si
// Ts·kd_i/m_i < ~1, con m_i la inercia efectiva del eje. En el robot real la
// inercia reflejada del rotor de cada Dynamixel la sube y admite las ganancias
// del ensayo de identificacion (hw_alpha_ident_node). Gazebo no la simula: sin
// ella, la inercia efectiva de J3/J4 baja a ~0.4 y ~0.2 g·m² (acoplamiento
// J3–J4) y esas mismas ganancias darian un ciclo limite, como el de K_D4 = 0.10
// documentado en gz_MRAC_joint_12p_node. Por eso en simulacion J3/J4 usan
// ganancias menores.
#pragma once

#include "omx/robot_model.hpp"
#include "omx/trajectory.hpp"
#include "omx/types.hpp"

namespace omx
{

class JointReturn
{
public:
  // duration: duracion de la quintica [s]; despues la referencia queda en reposo
  // en la pose inicial. kp [N·m/rad], kd [N·m·s/rad].
  JointReturn(double duration, const Vec4 & kp, const Vec4 & kd)
  : duration_(duration), kp_(kp), kd_(kd)
  {
  }

  // Robot real: ganancias del ensayo de identificacion, probadas en el robot
  // (con la inercia del URDF en reposo, ωn ≈ 15–50 rad/s y ζ ≈ 0.5–0.9)
  static JointReturn forHardware(double duration)
  {
    return JointReturn(duration, Vec4(2.5, 5.0, 5.0, 3.0), Vec4(0.17, 0.33, 0.28, 0.08));
  }

  // Gazebo: J1/J2 iguales; J3/J4 con ζ ≈ 0.7 en reposo (ωn ≈ 6 y 16 rad/s) y
  // margen de ganancia ~1.5 sobre el limite de estabilidad del lazo discreto
  static JointReturn forSimulation(double duration)
  {
    return JointReturn(duration, Vec4(2.5, 5.0, 0.17, 0.3), Vec4(0.17, 0.33, 0.04, 0.025));
  }

  // Pose articular a la que vuelve el brazo (la medida en el primer tick)
  void setHome(const Vec4 & q_home) { q_home_ = q_home; }

  // Primer tick del retorno: la quintica parte del estado medido, sin salto de error
  void start(const Vec4 & q, const Vec4 & dq)
  {
    start_ = Reference{q, dq, Vec4::Zero()};
    started_ = true;
  }
  bool started() const { return started_; }

  // Referencia a t segundos del inicio del retorno (reposo en q_home desde t = duration)
  Reference reference(double t) const { return quinticReturn(t, duration_, start_, q_home_); }

  Vec4 torque(RobotModel & model, const Vec4 & q, const Vec4 & dq, const Reference & ref) const
  {
    return model.gravity(q) + kp_.cwiseProduct(ref.q - q) + kd_.cwiseProduct(ref.dq - dq);
  }

private:
  double duration_;
  Vec4 kp_, kd_;
  Vec4 q_home_{Vec4::Zero()};
  Reference start_{Vec4::Zero(), Vec4::Zero(), Vec4::Zero()};
  bool started_{false};
};

}  // namespace omx
