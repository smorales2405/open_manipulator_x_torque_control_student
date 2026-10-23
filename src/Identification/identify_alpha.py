#!/usr/bin/env python3
"""
identify_alpha.py
Identificación de alpha (torque→corriente) e I_offset de los 4 motores
Dynamixel XM430-W350-T del OpenMANIPULATOR-X: GANANCIA DEL LAZO DE CORRIENTE.

Cadena que usan todos los nodos hw_* (modo corriente):

    tau ──(alpha)──▶ I_cmd (Goal Current) ──(lazo interno del Dynamixel)──▶ I ──(kt)──▶ torque

El lazo interno se ajusta con los datos:  I_meas ≈ k·I_cmd + b,  y de ahí

    alpha    = ALPHA_NOM / k       ALPHA_NOM = 1/(kt_ds·Iu) ≈ 208.5 ticks/N·m
    I_offset = −b / k              (cancela el offset del lazo)

con kt_ds = 4.1 N·m / 2.3 A = 1.78 N·m/A (punto de stall del datasheet a 12 V,
en el eje de salida) e Iu = 2.69 mA/tick. Con k = 1 y b = 0 el driver entrega
exactamente la corriente pedida y alpha = ALPHA_NOM.

QUÉ NO SE IDENTIFICA (y por qué): el kt real. Con movimiento y corriente, kt
aparece multiplicado por las masas/inercias del URDF (I = alpha·Y(q,dq,ddq)·θ:
escalar alpha por c y θ por 1/c deja la corriente igual), así que separarlo
exige una carga de masa conocida. alpha se ancla al datasheet; validación
cruzada en hardware: con alpha ≈ 207–209 el MRAC 12p dejó de mostrar la deriva
que tenía con alpha = 224.8 (su parámetro α₃ quedó estable en ≈ 1.01).

POR QUÉ CAMBIÓ EL MÉTODO: la versión anterior ajustaba I_meas contra el TORQUE
comandado. Como el nodo calcula I_cmd = alpha_usada·tau, esa regresión solo
devuelve el alpha que usó el nodo multiplicado por k: con alpha = 224.8 en el
YAML devolvía ≈ 225, y con la conversión nominal devolvió 207.1
(= 208.5 × 0.993). No medía kt; hacía eco del modelo del nodo. Ajustar I_meas
contra I_cmd mide k y b directamente, no depende del alpha ni de la
compensación que usó el nodo (no hay circularidad) y sirve con cualquier
ensayo en modo corriente.

Detalles del ajuste:
    · Alineación de --shift muestras (def. 1): los nodos leen el estado ANTES
      de escribir la corriente del ciclo, así que I_meas de la fila n+1 es la
      respuesta a I_cmd de la fila n.
    · Pasa-bajos de fase cero (Butterworth 2º orden, --fc 2 Hz) a ambas
      señales: el lazo se evalúa en la banda en que operan los controladores;
      la cuantización y el stick-slip de alta frecuencia no sesgan k.
    · Se descartan las muestras con corriente comandada saturada (±0.25 s).
    · k propio solo si el joint cumple las tres condiciones:
        - I_cmd excitada: std ≥ 6 ticks;
        - corr(I_cmd, I_meas) ≥ 0.95;
        - sesgo por cruce por cero ≤ 2 %. El lazo entrega ~0.4 ticks MENOS de
          lo pedido, con el signo de la corriente (medido en 23 logs reales:
          0.1–0.45 ticks). Si la corriente es chica y cambia de signo a menudo,
          ese desfase baja la pendiente: J1, sin gravedad y con ±10 ticks de
          fricción, da k ≈ 0.92–0.97 aunque su lazo esté bien. El sesgo se
          estima como 0.4·cov(I_cmd, sgn I_cmd)/var(I_cmd); si la corriente no
          cambia de signo es ~0 (el desfase queda en b).
      Con alpha_ident.launch.py: J2 (gravedad), J3 (fricción) y J4 (fricción y
      gravedad del efector, oscila ±0.25 rad) → k propio; J1 → k compartido =
      media de los excitados (mismo motor y firmware), b propio.

Veredicto (sobre los joints excitados):
    |k−1| ≤ 3 %  → OK
    3–10 %       → REVISAR (también: |b| > 3 ticks, dispersión de k > 3 %,
                   R² < 0.95, saturación > 2 %)
    > 10 %       → RE-EJECUTAR: el lazo no sigue la corriente pedida
                   (alimentación, Current Limit, modo de operación)

Entrada (autodetectada en este orden; --csv para forzar una ruta):
    data/identification/alpha/hw_alpha_ident_<ID>.csv    (alpha_ident.launch.py) ← recomendado
    data/feedback_linearization/real/act1/hw_fl_data_<ID>.csv  (hw_fl_control_node, cualquier modelo)
    data/diagnostics/sinusoidal/hw_sin_torque_<ID>.csv   (joints en mode="current")

Salidas:
    1) Consola : k, b, alpha, I_offset, R² por joint y VEREDICTO.
    2) YAML    : data/identification/alpha_fit_log<ID>.yaml  (fragmento para
                 assemble_motor_params.py)
    3) CSV     : data/identification/alpha_fit_log<ID>.csv   (t, I_cmd, I_meas
                 e I_pred filtradas por joint, para graficar el ajuste)

Código de salida: 0 = OK/REVISAR, 2 = RE-EJECUTAR.

Requisitos: numpy, scipy, PyYAML (no requiere Pinocchio).
Uso:
    python3 src/Identification/identify_alpha.py 30
    python3 src/Identification/identify_alpha.py --csv /ruta/al/log.csv
Protocolo completo: src/Identification/Ident_OpenManX_XM430W350T_procedure.md
"""

import os
import sys
import csv
import argparse
import datetime
import numpy as np
import yaml
from scipy.signal import butter, filtfilt

# ════════════════════════════════════════════════════════════════════════════
#  Configuración
# ════════════════════════════════════════════════════════════════════════════

# Raíz del paquete: dos niveles arriba de src/Identification/
PKG_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
NJ = 4

# Constantes del XM430-W350
CURRENT_UNIT_A = 2.69e-3          # [A/tick]
KT_DATASHEET   = 1.7826           # [N·m/A] 4.1 N·m / 2.3 A (stall, 12 V)
ALPHA_NOM      = 1.0 / (KT_DATASHEET * CURRENT_UNIT_A)   # ≈ 208.5 ticks/N·m

# Identificabilidad del lazo por joint (sobre las señales filtradas)
CMD_STD_MIN = 6.0                 # [ticks] excitación mínima de I_cmd
CORR_MIN    = 0.95                # corr(I_cmd, I_meas) mínima
SIGN_OFFSET = 0.4                 # [ticks] desfase del lazo con el signo de I (típico)
ZC_BIAS_MAX = 0.02                # sesgo de k por cruce por cero máximo

# Veredicto
K_OK      = 0.03                  # |k−1| ≤ 3 %  → OK
K_FAIL    = 0.10                  # |k−1| > 10 % → RE-EJECUTAR
K_SPREAD  = 0.03                  # dispersión de k entre joints excitados
B_WARN    = 3.0                   # [ticks] |b| sospechoso
R2_WARN   = 0.95

# Saturación de corriente comandada (current_cmd_limit de los nodos)
CURRENT_CMD_LIMIT = 257           # [ticks]
SAT_GUARD_S       = 0.25          # [s] margen descartado alrededor de la saturación
SAT_FRAC_WARN     = 0.02

# ════════════════════════════════════════════════════════════════════════════
#  CLI y carga de datos
# ════════════════════════════════════════════════════════════════════════════

ap = argparse.ArgumentParser(description="Identificación de alpha e I_offset "
                             "(ganancia del lazo de corriente).")
ap.add_argument("log_id", nargs="?", type=int, default=None,
                help="ID del log (hw_alpha_ident_<id>, hw_fl_data_<id> o hw_sin_torque_<id>)")
ap.add_argument("--csv",   default=None, help="ruta explícita al CSV de entrada")
ap.add_argument("--skip",  type=float, default=4.0,
                help="segundos a descartar al inicio (rampa/transitorio; def. 4.0)")
ap.add_argument("--shift", type=int, default=1,
                help="muestras entre I_cmd y su I_meas (def. 1: lectura antes de escritura)")
ap.add_argument("--fc",    type=float, default=2.0,
                help="corte del pasa-bajos de fase cero [Hz] (def. 2.0)")
args = ap.parse_args()

if args.csv is None and args.log_id is None:
    ap.error("indique LOG_ID o --csv")

if args.csv is not None:
    csv_in = args.csv
else:
    cands = [
        os.path.join(PKG_DIR, "data", "identification", "alpha",
                     f"hw_alpha_ident_{args.log_id}.csv"),
        os.path.join(PKG_DIR, "data", "feedback_linearization", "real", "act1",
                     f"hw_fl_data_{args.log_id}.csv"),
        os.path.join(PKG_DIR, "data", "diagnostics", "sinusoidal",
                     f"hw_sin_torque_{args.log_id}.csv"),
    ]
    found = [c for c in cands if os.path.isfile(c)]
    if not found:
        raise FileNotFoundError(
            "No se encontró el log en ninguna de las rutas esperadas:\n  "
            + "\n  ".join(cands))
    csv_in = found[0]
    if len(found) > 1:
        print("Aviso: hay más de un log con ese ID; se usa el primero (--csv para elegir):")
        for c in found:
            print(("  → " if c == csv_in else "    ") + c)

# Signos de hardware: I_cmd/I_meas del CSV están en el dominio crudo del
# Dynamixel; I_offset se expresa en el dominio articular del modelo.
SGN = np.ones(NJ)
cfg_path = os.path.join(PKG_DIR, "config", "motorXM430W350T_params.yaml")
if os.path.isfile(cfg_path):
    with open(cfg_path) as f:
        prm = (yaml.safe_load(f) or {}).get("/**", {}).get("ros__parameters", {})
    SGN = (np.asarray(prm.get("current_sign", [1.0] * NJ), float)
           * np.asarray(prm.get("encoder_sign", [1.0] * NJ), float))

raw = np.genfromtxt(csv_in, delimiter=",", names=True)
cols = raw.dtype.names
t = raw["t"] - raw["t"][0]
N = len(t)
Ts = float(np.mean(np.diff(t)))

if not all(f"curr_cmd{i}" in cols and f"curr_meas{i}" in cols for i in range(1, NJ + 1)):
    raise RuntimeError("El CSV no tiene columnas curr_cmd1..4 / curr_meas1..4 — "
                       "¿es un log de un nodo hw_ en modo corriente?")
cmd_raw  = np.column_stack([raw[f"curr_cmd{i}"]  for i in range(1, NJ + 1)])
meas_raw = np.column_stack([raw[f"curr_meas{i}"] for i in range(1, NJ + 1)])

print(f"Cargado : {csv_in}")
print(f"  {N} muestras | Ts = {Ts*1000:.2f} ms | fs = {1/Ts:.1f} Hz")

# ── Debe haber corriente comandada (modo corriente) ─────────────────────────
has_cmd = np.abs(cmd_raw).max(axis=0) > 0
if not has_cmd.any():
    print("\n" + "!" * 78)
    print(" ERROR: ningún joint tiene corriente comandada (curr_cmd = 0 en todo el log).")
    print(" El ensayo no se hizo en modo corriente (p.ej. barrido en velocity/position).")
    print(" → Acción: ejecute el ensayo de alpha (Paso 2 del protocolo):")
    print("     ros2 launch open_manipulator_x_torque_control alpha_ident.launch.py log_id:=<ID>")
    print("!" * 78)
    sys.exit(2)

# ── Alineación: I_meas[n+shift] responde a I_cmd[n] ─────────────────────────
s = max(args.shift, 0)
M = N - s
x_raw = cmd_raw[:M]
y_raw = meas_raw[s:]
tt    = t[:M]

# ── Ventana: desde el inicio del control + skip, sin saturación ─────────────
act = np.where(np.any(np.abs(x_raw) > 0, axis=1))[0]
t0 = tt[act[0]] if act.size else 0.0
win = tt >= (t0 + args.skip)
if int(win.sum()) < 400:
    print(f"\nERROR: solo {int(win.sum())} muestras tras descartar {args.skip:.1f} s — "
          "log demasiado corto.")
    print("→ Acción: repita el ensayo con t_run ≥ 30 s.")
    sys.exit(2)
print(f"  Ventana de análisis: t ≥ {t0 + args.skip:.2f} s → {int(win.sum())} muestras "
      f"| alineación {s} muestra(s) | pasa-bajos {args.fc:.1f} Hz")

guard = max(1, int(round(SAT_GUARD_S / Ts)))
kernel = np.ones(2 * guard + 1)
mask_j, sat_frac = [], np.zeros(NJ)
for i in range(NJ):
    sat = np.abs(x_raw[:, i]) >= (CURRENT_CMD_LIMIT - 0.5)
    sat_frac[i] = float(sat[win].mean())
    near_sat = np.convolve(sat.astype(float), kernel, mode="same") > 0
    mask_j.append(win & ~near_sat)

# ── Filtro de fase cero (misma respuesta en ambas señales) ──────────────────
b_lp, a_lp = butter(2, args.fc / (0.5 / Ts), "low")
x_f = filtfilt(b_lp, a_lp, x_raw, axis=0)
y_f = filtfilt(b_lp, a_lp, y_raw, axis=0)

# ════════════════════════════════════════════════════════════════════════════
#  Ajuste del lazo por joint:  I_meas = k·I_cmd + b
# ════════════════════════════════════════════════════════════════════════════

k_own = np.full(NJ, np.nan); b_own = np.full(NJ, np.nan)
cmd_std = np.zeros(NJ); corr = np.zeros(NJ); zc_bias = np.zeros(NJ)
for i in range(NJ):
    m = mask_j[i]
    xi, yi = x_f[m, i], y_f[m, i]
    cmd_std[i] = float(np.std(xi))
    if cmd_std[i] > 1e-6 and np.std(yi) > 1e-6:
        corr[i] = float(np.corrcoef(xi, yi)[0, 1])
        # Pendiente que perdería k por un desfase de SIGN_OFFSET·sgn(I_cmd)
        zc_bias[i] = SIGN_OFFSET * float(np.cov(xi, np.sign(xi))[0, 1]) / cmd_std[i] ** 2
        A = np.column_stack([xi, np.ones(xi.size)])
        (k_own[i], b_own[i]), *_ = np.linalg.lstsq(A, yi, rcond=None)

strong = [i for i in range(NJ) if cmd_std[i] >= CMD_STD_MIN and corr[i] >= CORR_MIN
          and zc_bias[i] <= ZC_BIAS_MAX]
weak   = [i for i in range(NJ) if i not in strong]

k = np.full(NJ, np.nan); b = np.zeros(NJ)
for i in strong:
    k[i], b[i] = k_own[i], b_own[i]
k_shared = float(np.mean(k[strong])) if strong else float("nan")
for i in weak:
    k[i] = k_shared
    m = mask_j[i]
    b[i] = float(np.mean(y_f[m, i] - k_shared * x_f[m, i])) if has_cmd[i] else 0.0

alpha = ALPHA_NOM / k
I_off = -SGN * b / k

# R² (señales filtradas) y residuo RMS (crudo alineado) por joint
R2 = np.full(NJ, np.nan); resid_rms = np.full(NJ, np.nan)
I_pred = np.full((M, NJ), np.nan)
for i in range(NJ):
    if not has_cmd[i] or not np.isfinite(k[i]):
        continue
    m = mask_j[i]
    pred_f = k[i] * x_f[:, i] + b[i]
    ss_tot = float(((y_f[m, i] - y_f[m, i].mean()) ** 2).sum())
    if ss_tot > 0:
        R2[i] = 1.0 - float(((y_f[m, i] - pred_f[m]) ** 2).sum()) / ss_tot
    resid_rms[i] = float(np.sqrt(np.mean((y_raw[m, i] - (k[i] * x_raw[m, i] + b[i])) ** 2)))
    I_pred[m, i] = pred_f[m]

# ════════════════════════════════════════════════════════════════════════════
#  Reporte
# ════════════════════════════════════════════════════════════════════════════

print("\n" + "=" * 104)
print(f" alpha / I_offset — lazo de corriente I_meas = k·I_cmd + b   [{os.path.basename(csv_in)}]")
print("=" * 104)
print(f"{'Joint':>6} {'k':>8} {'b':>7} {'alpha':>9} {'I_offset':>9} {'R²':>7} "
      f"{'std(Icmd)':>10} {'corr':>6} {'sesgo0%':>8} {'sat%':>5}  fuente")
print("-" * 104)
for i in range(NJ):
    if not has_cmd[i]:
        fuente = "sin corriente comandada"
    elif i in strong:
        fuente = "k propio"
    else:
        why = []
        if cmd_std[i] < CMD_STD_MIN:  why.append("poco excitado")
        if corr[i] < CORR_MIN:        why.append("corr baja")
        if zc_bias[i] > ZC_BIAS_MAX:  why.append("cruza cero")
        own = f" (propio {k_own[i]:.3f}, {', '.join(why)})" if np.isfinite(k_own[i]) else ""
        fuente = "k compartido" + own
    print(f"J{i+1:>5} {k[i]:8.4f} {b[i]:7.2f} {alpha[i]:9.2f} {I_off[i]:9.2f} {R2[i]:7.4f} "
          f"{cmd_std[i]:10.1f} {corr[i]:6.3f} {100*zc_bias[i]:8.1f} {100*sat_frac[i]:5.1f}  {fuente}")
print("-" * 104)
print(f"  alpha = ALPHA_NOM/k con ALPHA_NOM = {ALPHA_NOM:.2f} ticks/N·m "
      f"(kt datasheet {KT_DATASHEET:.4f} N·m/A); I_offset = −b/k")
print(f"  k propio si std(Icmd) ≥ {CMD_STD_MIN:.0f} ticks, corr ≥ {CORR_MIN} y sesgo0 "
      f"≤ {100*ZC_BIAS_MAX:.0f} % (sesgo de k por cruce por cero)")
print(f"  k compartido (joints débiles) = {k_shared:.4f}.  Unidades: b, I_offset [ticks]")

# ── Veredicto ────────────────────────────────────────────────────────────────
fails, warns = [], []

if not strong:
    fails.append(
        f"Ningún joint con corriente comandada bien excitada (std ≥ {CMD_STD_MIN:.0f} ticks,\n"
        f"   corr ≥ {CORR_MIN} y sesgo0 ≤ {100*ZC_BIAS_MAX:.0f} %).\n"
        "   → Acción: verifique que el robot realmente se movió (¿parada de emergencia?)\n"
        "     y repita con alpha_ident.launch.py y t_run ≥ 30 s.")
else:
    for i in strong:
        dk = abs(k[i] - 1.0)
        if dk > K_FAIL:
            fails.append(
                f"J{i+1}: k = {k[i]:.3f} (|k−1| > {100*K_FAIL:.0f} %): el lazo de corriente NO sigue\n"
                "   la corriente pedida. Revise alimentación (12 V estable bajo carga),\n"
                "   current_limit_register del YAML, temperatura de motores, y que los\n"
                "   servos estén en modo corriente (Operating Mode 0). Luego repita.")
        elif dk > K_OK:
            warns.append(f"J{i+1}: k = {k[i]:.3f} (|k−1| = {100*dk:.1f} % > {100*K_OK:.0f} %): "
                         "lazo de corriente algo desviado;\n"
                         "   alpha corregido utilizable. Revise alimentación/temperatura.")
        if R2[i] < R2_WARN:
            warns.append(f"J{i+1}: R² = {R2[i]:.3f} < {R2_WARN}: relación I_cmd↔I_meas poco lineal "
                         "(¿saturación, golpes, cableado?).")
    if len(strong) > 1:
        spread = float(np.max(k[strong]) - np.min(k[strong]))
        if spread > K_SPREAD:
            warns.append(f"Dispersión de k entre joints excitados = {spread:.3f} (> {K_SPREAD}): "
                         "algún motor\n   se aparta del resto (temperatura, firmware, cableado).")
for i in range(NJ):
    if has_cmd[i] and abs(b[i]) > B_WARN:
        warns.append(f"J{i+1}: |b| = {abs(b[i]):.1f} ticks > {B_WARN:.0f}: offset del lazo/sensor "
                     "inusual (¿motor caliente, sensor descalibrado?).")
    if sat_frac[i] > SAT_FRAC_WARN:
        warns.append(f"J{i+1}: {100*sat_frac[i]:.1f}% de muestras con corriente saturada "
                     "(descartadas).\n   → Si supera ~10 %, repita con gain_scale:=0.7.")

print("\n" + "=" * 104)
if fails:
    print(" VEREDICTO: RE-EJECUTAR — los datos NO sirven para identificar alpha")
    for f_ in fails:
        print(" ✗ " + f_)
    for w in warns:
        print(" ! " + w)
    print("=" * 104)
    sys.exit(2)
elif warns:
    print(" VEREDICTO: REVISAR — alpha utilizable, pero con observaciones")
    for w in warns:
        print(" ! " + w)
else:
    print(" VEREDICTO: OK — el lazo de corriente sigue lo comandado; alpha e I_offset listos")
print("   Siguiente paso: python3 src/Identification/assemble_motor_params.py "
      "--alpha-log <este log> --friction-logs <J1> <J2> <J3> <J4>")
print("=" * 104)

# ════════════════════════════════════════════════════════════════════════════
#  Salidas: fragmento YAML + CSV de ajuste
# ════════════════════════════════════════════════════════════════════════════

out_dir = os.path.join(PKG_DIR, "data", "identification")
os.makedirs(out_dir, exist_ok=True)
log_tag = args.log_id if args.log_id is not None else \
          os.path.splitext(os.path.basename(csv_in))[0]

yaml_out = os.path.join(out_dir, f"alpha_fit_log{log_tag}.yaml")
fmt = lambda v: "[" + ", ".join(f"{x:.4f}" for x in v) + "]"
with open(yaml_out, "w") as f:
    f.write("# Fragmento de identificación de alpha/I_offset (identify_alpha.py)\n")
    f.write(f"# Fecha: {datetime.date.today().isoformat()}   Fuente: {csv_in}\n")
    f.write("# Método: lazo de corriente I_meas = k·I_cmd + b ; "
            "alpha = ALPHA_NOM/k ; I_offset = -b/k\n")
    f.write(f"source_csv: {os.path.basename(csv_in)}\n")
    f.write(f"alpha:            {fmt(alpha)}\n")
    f.write(f"I_offset:         {fmt(I_off)}\n")
    f.write(f"loop_gain:        {fmt(k)}\n")
    f.write(f"loop_offset:      {fmt(b)}\n")
    f.write(f"loop_gain_shared: {k_shared:.4f}\n")
    f.write(f"alpha_nominal:    {ALPHA_NOM:.4f}\n")
    f.write(f"kt_datasheet:     {KT_DATASHEET:.4f}\n")
    f.write(f"strong_joints:    {[i + 1 for i in strong]}\n")
    f.write(f"cmd_std_ticks:    {fmt(cmd_std)}\n")
    f.write(f"zero_cross_bias:  {fmt(zc_bias)}\n")
    f.write(f"R2:               {fmt(np.nan_to_num(R2))}\n")
    f.write(f"resid_rms_ticks:  {fmt(np.nan_to_num(resid_rms))}\n")
    f.write(f"shift_samples:    {s}\n")
    f.write(f"lowpass_hz:       {args.fc:.2f}\n")
    f.write(f"verdict: {'REVISAR' if warns else 'OK'}\n")
print(f"\nFragmento YAML : {yaml_out}")

fit_out = os.path.join(out_dir, f"alpha_fit_log{log_tag}.csv")
header = (["t"] + [f"I_cmd{i+1}" for i in range(NJ)]
          + [f"I_meas{i+1}" for i in range(NJ)]
          + [f"I_pred{i+1}" for i in range(NJ)])
with open(fit_out, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(header)
    for n in np.where(win)[0]:
        w.writerow([f"{tt[n]:.6f}"]
                   + [f"{x_f[n, i]:.4f}"   for i in range(NJ)]
                   + [f"{y_f[n, i]:.4f}"   for i in range(NJ)]
                   + [f"{I_pred[n, i]:.4f}" for i in range(NJ)])
print(f"CSV de ajuste  : {fit_out}")
