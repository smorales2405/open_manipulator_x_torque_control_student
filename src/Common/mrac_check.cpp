// Implementacion de omx::checkMrac12p (ver include/omx/mrac_check.hpp).
#include "omx/mrac_check.hpp"

#include <algorithm>
#include <cstdio>

namespace omx
{

namespace
{

constexpr double TOL = 1e-8;   // [N·m] las dos formas deben coincidir a precision de maquina

struct TestState { Vec4 q, dq, dqr, ddqr, th; };

double maxAbs(const Eigen::MatrixXd & m)
{
  return m.size() == 0 ? 0.0 : m.cwiseAbs().maxCoeff();
}

// Linea del informe: OK, sin completar (bloque en cero) o ERROR con la diferencia
std::string line(const char * name, double err, bool is_zero, const char * hint, bool & ok)
{
  char buf[256];
  if (err <= TOL) {
    std::snprintf(buf, sizeof(buf), "  %-46s OK\n", name);
  } else {
    ok = false;
    if (is_zero) {
      std::snprintf(buf, sizeof(buf), "  %-46s SIN COMPLETAR\n", name);
    } else {
      std::snprintf(buf, sizeof(buf), "  %-46s ERROR (max |dif| = %.3g)%s%s\n",
                    name, err, hint[0] ? ": " : "", hint);
    }
  }
  return buf;
}

}  // namespace

MracCheckResult checkMrac12p(RobotModel & model, const std::array<int, NUM_JOINTS> & jid,
                             MracNominal form, const Vec4 & fv_prior, const Vec4 & fc_prior,
                             const MracTorqueFn & tau_nom, const MracRegressorFn & regressor)
{
  // Estados de prueba: dq != dq_r para que C(q,dq)·dq_r se distinga de C(q,dq)·dq
  const std::array<TestState, 3> states = {{
    {Vec4(0.3, 0.5, -0.4, 0.8),  Vec4(0.6, -0.4, 0.7, -0.5), Vec4(-0.2, 0.9, -0.3, 0.6),
     Vec4(1.1, -0.7, 0.5, -1.3), Vec4(0.6, -0.3, 0.9, -0.7)},
    {Vec4(-0.7, -0.2, 0.6, -0.3), Vec4(-0.3, 0.8, -0.6, 0.4), Vec4(0.5, -0.4, 0.2, -0.9),
     Vec4(-0.6, 1.2, -0.8, 0.4), Vec4(-0.4, 0.8, -0.2, 0.5)},
    {Vec4(0.0, 0.931, 0.35, -1.49), Vec4(0.2, 0.1, -0.3, 0.6), Vec4(0.1, 0.3, -0.5, 0.2),
     Vec4(0.3, 0.2, -0.1, 0.7),  Vec4(0.95, -0.95, 0.1, 0.0)},
  }};

  std::array<RobotModel::Vec10, 3> pi;
  for (int i = 0; i < 3; ++i) pi[i] = model.bodyDynamicParameters(jid[i]);

  double e_nom = 0.0, e_alpha = 0.0, e_load = 0.0, e_fv = 0.0, e_fc = 0.0;
  double e_nom_rnea = 0.0, e_nom_sl = 0.0, e_nom_nofric = 0.0;
  bool z_nom = true, z_alpha = true, z_load = true, z_fv = true, z_fc = true;

  for (const auto & st : states) {
    const Vec4 fric = fv_prior.cwiseProduct(st.dq) + fc_prior.cwiseProduct(st.th);
    const Vec4 sl   = model.slotineLiTorque(st.q, st.dq, st.dqr, st.ddqr);
    const Vec4 rnea = model.inverseDynamics(st.q, st.dq, st.ddqr);
    const Vec4 ref  = (form == MracNominal::SlotineLi ? sl : rnea) + fric;

    const Vec4 tn = tau_nom(st.q, st.dq, st.dqr, st.ddqr, st.th);
    e_nom        = std::max(e_nom, maxAbs(tn - ref));
    e_nom_rnea   = std::max(e_nom_rnea, maxAbs(tn - (rnea + fric)));
    e_nom_sl     = std::max(e_nom_sl, maxAbs(tn - (sl + fric)));
    e_nom_nofric = std::max(e_nom_nofric, maxAbs(tn - (ref - fric)));
    z_nom = z_nom && maxAbs(tn) == 0.0;

    // Regresor de referencia
    const Eigen::MatrixXd Y_SL = model.slotineLiRegressor(st.q, st.dq, st.dqr, st.ddqr);
    MatY12 Yr = MatY12::Zero();
    for (int i = 0; i < 3; ++i) {
      Yr.col(i) = (Y_SL.middleCols(10 * (jid[i] - 1), 10) * pi[i]).head<NUM_JOINTS>();
    }
    Yr.block<NUM_JOINTS, 4>(0, 3) = Y_SL.block(0, 10 * (jid[3] - 1), NUM_JOINTS, 4);
    Yr(0, 7) = st.dq[0];
    for (int i = 0; i < NUM_JOINTS; ++i) Yr(i, 8 + i) = st.th[i];

    const MatY12 Ys = regressor(st.q, st.dq, st.dqr, st.ddqr, st.th);
    e_alpha = std::max(e_alpha, maxAbs(Ys.middleCols<3>(0) - Yr.middleCols<3>(0)));
    e_load  = std::max(e_load,  maxAbs(Ys.middleCols<4>(3) - Yr.middleCols<4>(3)));
    e_fv    = std::max(e_fv,    maxAbs(Ys.col(7) - Yr.col(7)));
    e_fc    = std::max(e_fc,    maxAbs(Ys.middleCols<4>(8) - Yr.middleCols<4>(8)));
    z_alpha = z_alpha && maxAbs(Ys.middleCols<3>(0)) == 0.0;
    z_load  = z_load  && maxAbs(Ys.middleCols<4>(3)) == 0.0;
    z_fv    = z_fv    && maxAbs(Ys.col(7)) == 0.0;
    z_fc    = z_fc    && maxAbs(Ys.middleCols<4>(8)) == 0.0;
  }

  // Pistas para los errores frecuentes del torque nominal
  const char * hint_nom = "";
  if (form == MracNominal::SlotineLi && e_nom_rnea <= TOL) {
    hint_nom = "usa C(q,dq)*dq; Slotine-Li usa C(q,dq)*dq_r";
  } else if (form == MracNominal::Rnea && e_nom_sl <= TOL) {
    hint_nom = "en hardware el termino de velocidad es n(q,dq)";
  } else if (fv_prior.cwiseAbs().maxCoeff() + fc_prior.cwiseAbs().maxCoeff() > 0.0 &&
             e_nom_nofric <= TOL) {
    hint_nom = "falta la friccion nominal";
  }

  bool ok = true;
  std::string rep;
  rep += line("tau_nom", e_nom, z_nom, hint_nom, ok);
  rep += line("regresor: columnas alpha1..alpha3", e_alpha, z_alpha, "", ok);
  rep += line("regresor: columna Fv1", e_fv, z_fv, "", ok);
  rep += line("regresor: columnas Fc1..Fc4", e_fc, z_fc, "", ok);
  rep += line("regresor: columnas de la carga (no modificar)", e_load, z_load, "", ok);
  return {ok, rep};
}

}  // namespace omx
