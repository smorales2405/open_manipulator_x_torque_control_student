#!/usr/bin/env python3
"""
update_urdf_friction.py
Copia la fricción identificada de config/motorXM430W350T_params.yaml al URDF
estático urdf/open_manipulator_x.urdf, en el dominio de TORQUE:

    damping  [N·m·s/rad] = motor_Fv / motor_alpha
    friction [N·m]       = motor_Fc / motor_alpha

(la misma conversión que hace xacro/open_manipulator_x.urdf.xacro). Solo
cambia los atributos de <dynamics> de joint1..joint4; el resto del URDF queda
intacto.

Por qué hace falta: el xacro de Gazebo lee el YAML en cada lanzamiento, así
que la planta simulada ya tiene la fricción nueva. Pero el URDF estático no
se regenera solo, y los nodos gz_* lo cargan con Pinocchio y compensan la
fricción con model.damping / model.friction: si queda con valores de otra
identificación, el controlador simulado compensa una fricción distinta de la
que tiene la planta.

Ejecutarlo DESPUÉS de activar el modelo nuevo (Paso 3 del protocolo):
    cp config/motorXM430W350T_params_identified.yaml config/motorXM430W350T_params.yaml
    python3 src/Identification/update_urdf_friction.py --dry-run   # muestra el cambio
    python3 src/Identification/update_urdf_friction.py             # escribe el URDF

Opciones: --params RUTA (YAML de entrada), --urdf RUTA (URDF destino),
          --force (acepta un YAML con Fv = Fc = 0, p.ej. bootstrap).
Los nodos leen el URDF del árbol fuente: no hace falta recompilar.
Código de salida: 0 = actualizado o sin cambios, 2 = error.
Protocolo completo: src/Identification/Ident_OpenManX_XM430W350T_procedure.md
"""

import os
import re
import sys
import argparse
import yaml

PKG_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
NJ = 4

ap = argparse.ArgumentParser(description="Copia Fv/Fc del YAML de motor al "
                             "<dynamics> de joint1..joint4 del URDF estático.")
ap.add_argument("--params", default=os.path.join(PKG_DIR, "config",
                "motorXM430W350T_params.yaml"), help="YAML de motor (entrada)")
ap.add_argument("--urdf", default=os.path.join(PKG_DIR, "urdf",
                "open_manipulator_x.urdf"), help="URDF a actualizar")
ap.add_argument("--dry-run", action="store_true",
                help="muestra el cambio sin escribir el URDF")
ap.add_argument("--force", action="store_true",
                help="acepta un YAML sin fricción (Fv = Fc = 0)")
args = ap.parse_args()

# ── Parámetros de motor → dominio de torque ──────────────────────────────────
with open(args.params) as f:
    prm = (yaml.safe_load(f) or {}).get("/**", {}).get("ros__parameters", {})
try:
    alpha = [float(x) for x in prm["motor_alpha"]]
    Fv    = [float(x) for x in prm["motor_Fv"]]
    Fc    = [float(x) for x in prm["motor_Fc"]]
except KeyError as e:
    print(f"ERROR: {args.params} no tiene {e}.")
    sys.exit(2)

if any(a <= 0 for a in alpha):
    print(f"ERROR: motor_alpha = {alpha} debe ser positivo.")
    sys.exit(2)
if all(v == 0 for v in Fv + Fc) and not args.force:
    print(f"ERROR: {os.path.basename(args.params)} tiene Fv = Fc = 0 en los 4 joints")
    print("  (¿YAML en modo bootstrap?). Copiarlo dejaría el URDF sin fricción.")
    print("→ Acción: active primero el modelo identificado (Paso 3 del protocolo):")
    print("     cp config/motorXM430W350T_params_identified.yaml config/motorXM430W350T_params.yaml")
    print("  o use --force si de verdad quiere fricción nula en el URDF.")
    sys.exit(2)

damping  = [Fv[i] / alpha[i] for i in range(NJ)]
friction = [Fc[i] / alpha[i] for i in range(NJ)]

# ── Reemplazo en el URDF ─────────────────────────────────────────────────────
with open(args.urdf) as f:
    text = f.read()

def attr(tag, name):
    m = re.search(rf'\b{name}="([^"]*)"', tag)
    return float(m.group(1)) if m else None

old = []
for j in range(1, NJ + 1):
    # El <joint> cinemático (con type=); el bloque ros2_control también se
    # llama jointN pero no tiene type.
    jm = re.search(rf'<joint\b(?=[^>]*\bname="joint{j}")(?=[^>]*\btype=")[^>]*>'
                   r'(.*?)</joint>', text, re.S)
    if jm is None:
        print(f"ERROR: no se encontró <joint name=\"joint{j}\" type=...> en {args.urdf}")
        sys.exit(2)
    new_tag = f'<dynamics damping="{damping[j-1]:.4f}" friction="{friction[j-1]:.4f}"/>'
    body = jm.group(1)
    dm = re.search(r'<dynamics\b[^>]*/>', body)
    if dm:
        old.append((attr(dm.group(0), "damping"), attr(dm.group(0), "friction")))
        new_body = body[:dm.start()] + new_tag + body[dm.end():]
    else:
        # Sin <dynamics>: se agrega antes de </joint> con la sangría de la
        # línea anterior.
        old.append((None, None))
        indent = re.search(r'\n([ \t]*)<[^\n]*\n[ \t]*$', body)
        ind = indent.group(1) if indent else "    "
        new_body = body.rstrip(" \t") + ind + new_tag + "\n" + body[len(body.rstrip(" \t")):]
    text = text[:jm.start(1)] + new_body + text[jm.end(1):]

# ── Reporte ──────────────────────────────────────────────────────────────────
fmt = lambda v: "   —  " if v is None else f"{v:.4f}"
print("=" * 78)
print(f" Fricción del URDF desde {os.path.relpath(args.params, PKG_DIR)}")
print("   damping = motor_Fv/motor_alpha [N·m·s/rad]   friction = motor_Fc/motor_alpha [N·m]")
print("=" * 78)
print(f" {'joint':<7} {'damping actual → nuevo':>26} {'friction actual → nuevo':>28}")
print("-" * 78)
changed = False
for j in range(NJ):
    d0, f0 = old[j]
    d1, f1 = round(damping[j], 4), round(friction[j], 4)
    mark = "" if (d0 == d1 and f0 == f1) else "   ← cambia"
    changed |= bool(mark)
    print(f" joint{j+1:<2} {fmt(d0):>12} → {d1:.4f}      {fmt(f0):>12} → {f1:.4f}{mark}")
print("-" * 78)

if not changed:
    print(" El URDF ya tiene estos valores: nada que escribir.")
elif args.dry_run:
    print(" --dry-run: URDF NO modificado. Ejecute sin --dry-run para escribirlo.")
else:
    with open(args.urdf, "w") as f:
        f.write(text)
    print(f" URDF actualizado: {args.urdf}")
    print(f" Revise el cambio con:  git diff '{args.urdf}'")
print("=" * 78)
