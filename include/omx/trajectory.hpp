// Trayectorias quinticas compartidas por todos los nodos:
//   quinticTransition — rampa inicial desde la pose medida hasta la trayectoria
//   quinticReturn     — retorno final desde la trayectoria hasta una pose en reposo
//   quinticBlend      — polinomio quintico general entre dos estados de borde
//   quinticProfile    — perfil escalar normalizado s(x) de 0 a 1 (rampas suaves)
// Las trayectorias existen en version articular (Reference) y cartesiana (CartRef).
#pragma once

#include <algorithm>

#include "omx/types.hpp"

namespace omx
{

// Perfil quintico normalizado s(x) = 10x³ − 15x⁴ + 6x⁵ con x saturado a [0, 1]:
// s(0) = 0 y s(1) = 1, con velocidad y aceleracion nulas en ambos extremos.
inline double quinticProfile(double x)
{
  x = std::min(std::max(x, 0.0), 1.0);
  return x * x * x * (10.0 + x * (-15.0 + 6.0 * x));
}

// Polinomio quintico que interpola (q, dq, ddq) desde el estado (x0, v0, a0) en
// t = 0 hasta (xf, vf, af) en t = T. Fuera de [0, T] entrega el estado final
// (t se satura en T); con T <= 0 entrega directamente el estado final.
inline Reference quinticBlend(double t, double T,
                              const Vec4 & x0, const Vec4 & v0, const Vec4 & a0,
                              const Vec4 & xf, const Vec4 & vf, const Vec4 & af)
{
  if (T <= 0.0) {
    return Reference{xf, vf, af};
  }
  const double tc = std::min(t, T);
  const double T2 = T * T, T3 = T2 * T, T4 = T3 * T, T5 = T4 * T;
  const Vec4 c0 = x0;
  const Vec4 c1 = v0;
  const Vec4 c2 = 0.5 * a0;
  const Vec4 c3 = (20.0 * (xf - x0) - (8.0 * vf + 12.0 * v0) * T - (3.0 * a0 - af) * T2) / (2.0 * T3);
  const Vec4 c4 = (30.0 * (x0 - xf) + (14.0 * vf + 16.0 * v0) * T + (3.0 * a0 - 2.0 * af) * T2) / (2.0 * T4);
  const Vec4 c5 = (12.0 * (xf - x0) - (6.0 * vf + 6.0 * v0) * T - (a0 - af) * T2) / (2.0 * T5);

  const double t2 = tc * tc, t3 = t2 * tc, t4 = t3 * tc, t5 = t4 * tc;
  Reference r;
  r.q   = c0 + c1 * tc + c2 * t2 + c3 * t3 + c4 * t4 + c5 * t5;
  r.dq  = c1 + 2.0 * c2 * tc + 3.0 * c3 * t2 + 4.0 * c4 * t3 + 5.0 * c5 * t4;
  r.ddq = 2.0 * c2 + 6.0 * c3 * tc + 12.0 * c4 * t2 + 20.0 * c5 * t3;
  return r;
}

// Rampa inicial: parte de q0 con velocidad v0 (por defecto en reposo) y
// aceleracion nula, y llega al estado goal en t = T con posicion, velocidad y
// aceleracion continuas en el empalme con la trayectoria.
inline Reference quinticTransition(double t, double T, const Vec4 & q0, const Reference & goal,
                                   const Vec4 & v0 = Vec4::Zero())
{
  return quinticBlend(t, T, q0, v0, Vec4::Zero(), goal.q, goal.dq, goal.ddq);
}

inline CartRef quinticTransition(double t, double T, const Vec4 & y0, const CartRef & goal,
                                 const Vec4 & v0 = Vec4::Zero())
{
  const Reference r = quinticBlend(t, T, y0, v0, Vec4::Zero(), goal.y, goal.ydot, goal.yddot);
  return CartRef{r.q, r.dq, r.ddq};
}

// Retorno final: parte del estado start (la trayectoria al terminar t_run) y
// llega en reposo (velocidad y aceleracion nulas) a qf en t = T.
inline Reference quinticReturn(double t, double T, const Reference & start, const Vec4 & qf)
{
  return quinticBlend(t, T, start.q, start.dq, start.ddq, qf, Vec4::Zero(), Vec4::Zero());
}

inline CartRef quinticReturn(double t, double T, const CartRef & start, const Vec4 & yf)
{
  const Reference r = quinticBlend(t, T, start.y, start.ydot, start.yddot, yf, Vec4::Zero(), Vec4::Zero());
  return CartRef{r.q, r.dq, r.ddq};
}

}  // namespace omx
