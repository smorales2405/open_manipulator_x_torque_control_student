# Protocolo de identificación torque→corriente — OpenMANIPULATOR-X (XM430-W350-T)

Este protocolo permite identificar, **en un robot OpenMANIPULATOR-X nuevo**, los
parámetros del modelo torque→corriente de los 4 motores principales (IDs 11–14)
y dejarlos listos en `config/motorXM430W350T_params.yaml`:

```
I[i] = alpha[i]·tau[i] + Fv[i]·dq[i] + Fc[i]·tanh(dq[i]/eps) + I_offset[i]
```

| Parámetro | Significado | Unidades |
|---|---|---|
| `motor_alpha` | ganancia torque→corriente comandada (α = 208.5/k: kt del datasheet corregido por la ganancia k del lazo de corriente) | ticks/N·m |
| `motor_Fv` | fricción viscosa | ticks/(rad/s) |
| `motor_Fc` | fricción de Coulomb | ticks |
| `motor_I_offset` | offset del lazo de corriente (−b/k) | ticks |
| `motor_epsilon_friction` | suavizado del tanh de Coulomb | rad/s |

**Pipeline completo** (2 tipos de ensayo + 4 scripts):

| Ensayo en el robot | Script de análisis | Produce |
|---|---|---|
| Barrido de velocidad constante (×4, uno por joint) — `friction_sweep.launch.py` | `identify_friction.py` | `Fv`, `Fc` por joint |
| PD + gravedad en modo corriente con conversión nominal fija — `alpha_ident.launch.py` | `identify_alpha.py` | `alpha`, `I_offset` |
| — | `assemble_motor_params.py` | YAML final ensamblado |
| — | `update_urdf_friction.py` | fricción del URDF estático (simulación) |

Ninguno de los dos ensayos necesita el nodo de Feedback Linearization: la
identificación puede hacerse **antes** del laboratorio de FL.

Cada script imprime un **VEREDICTO** (OK / OK con nota / REVISAR / RE-EJECUTAR)
con el comando concreto de re-ejecución cuando los datos no bastan. Código de
salida 2 = hay que repetir el ensayo.

---

## 1. Fundamento: un método por parámetro

Por qué cada parámetro exige SU método:

- **α e I_offset — ganancia del lazo de corriente.** Todos los nodos convierten
  el torque en corriente comandada (*Goal Current*), y el lazo interno del
  Dynamixel la convierte en corriente real:

  ```
  τ ──(α)──▶ I_cmd ──(lazo de corriente: I ≈ k·I_cmd + b)──▶ I ──(kt)──▶ torque
  ```

  El ensayo del Paso 2 comanda `I_cmd = α_nom·τ` y mide `I`. El ajuste da `k`
  y `b` por motor, y de ahí `α = α_nom/k` e `I_offset = −b/k`, con
  `α_nom = 1/(kt_ds·Iu) = 208.5 ticks/N·m`: `kt_ds = 4.1 N·m / 2.3 A =
  1.78 N·m/A` (punto de *stall* del datasheet XM430-W350 a 12 V, medido en el
  eje de salida) e `Iu = 2.69 mA/tick`. En un servo sano `k ≈ 1` y `b ≈ 0`,
  así que `α ≈ 208.5`.

  > **Qué NO se identifica: el kt real.** Con movimiento y corriente medida,
  > kt aparece siempre multiplicado por las masas e inercias del URDF
  > (`I = α·Y(q, q̇, q̈)·θ`: escalar α por c y θ por 1/c deja la corriente
  > igual). Separarlos exige una carga de masa conocida, por eso α se ancla al
  > datasheet. Validación cruzada en hardware: con α ≈ 207–209 el MRAC 12p
  > dejó de mostrar la deriva que tenía con α = 224.8 (su parámetro α₃ quedó
  > estable en ≈ 1.01).

  > **Métodos descartados (historial).**
  > - *Regresión de I_meas contra el torque comandado* (versión anterior de
  >   este protocolo, con el nodo FL): como el nodo calcula
  >   `I_cmd = α_usada·τ`, la regresión devuelve `α_usada × k`, un eco del
  >   modelo del nodo. El antiguo α ≈ 224.8 coincide con `1/(1.654·Iu)`, la
  >   kt = 1.654 N·m/A que ya traían los nodos: la regresión la devolvía, no
  >   la medía (de ahí el falso "kt_ef = 1.65, 7 % de pérdidas del
  >   reductor"). Con α = 208.5 la misma regresión devolvió
  >   207.1 = 208.5 × 0.993.
  > - *Modo posición / parte par del barrido contra G(q) del URDF*: dio
  >   α ≈ 153 (kt_ef = 2.43 N·m/A, imposible). Mezcla kt con el error de masas
  >   del URDF (G2 sobreestimada un 10–20 %) y con la fricción del reductor
  >   353.5:1, que en régimen cuasi-estático sostiene parte de la gravedad.

- **Fv y Fc — barrido de velocidad constante + reversión.** A velocidad
  constante (ddq = 0) y comparando corrientes a la MISMA posición en ambos
  sentidos, la gravedad (par en q) se cancela EXACTAMENTE y queda la fricción
  (impar). No requiere URDF ni α, y elimina la colinealidad Fv↔Fc que arruina
  el ajuste con trayectorias suaves.

---

## 2. Requisitos previos

1. **Workspace compilado** (desde `~/open_manx_ws`):

   ```bash
   cd ~/open_manx_ws
   CMAKE_BUILD_PARALLEL_LEVEL=2 nice -n 15 colcon build --symlink-install \
     --executor sequential --packages-select open_manipulator_x_torque_control
   source install/setup.bash
   ```

2. **Robot montado y despejado**: base firmemente sujeta, sin objetos en el
   volumen de trabajo, gripper montado (las masas del URDF lo incluyen).
   Alimentación 12 V estable y U2D2/USB conectado (`/dev/ttyUSB0`).

3. **Latencia del FTDI a 1 ms** (necesario para lazos ≥100 Hz). Instalar una sola
   vez por PC la regla udev de ROBOTIS, que la fija en cada conexión del U2D2
   (persiste al reconectar el USB y al reiniciar):

   ```bash
   ros2 run open_manipulator_x_bringup create_udev_rules   # una vez por PC (pide sudo)
   cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer   # debe imprimir 1
   ```

4. **Permisos del puerto**: usuario en el grupo `dialout`
   (`sudo usermod -aG dialout $USER` + re-login) o `sudo chmod 666 /dev/ttyUSB0`.

5. **Servos**: IDs 11–14, Protocol 2.0, baudrate 1 000 000 (configuración de
   fábrica del OpenMANIPULATOR-X).

6. **Python**: `numpy`, `scipy`, `matplotlib`, `PyYAML` (los trae ROS 2 Humble).
   Los scripts de análisis no necesitan MATLAB ni Pinocchio.

7. **Seguridad**: los dos launch del protocolo (`friction_sweep.launch.py` y
   `alpha_ident.launch.py`) tienen `open_port:=true` por defecto (el de
   fricción también `enable_torque` y `enable_current_commands`): **lanzarlos
   mueve el robot real**. Para un dry-run (verificar parámetros sin hardware)
   agregue `open_port:=false`. Todos los nodos se detienen ante corriente
   medida > `current_measured_peak` (313 ticks) y con `Ctrl+C` (torque off).
   Al terminar normalmente, los barridos y el ensayo de α **regresan solos** a
   la pose en que estaba el robot al lanzarlos antes de cortar el torque; en
   cambio `Ctrl+C` o una parada de emergencia cortan el torque **en el acto**
   (el brazo cae por gravedad: sosténgalo). Tenga siempre a mano el corte de
   alimentación.

> Los CSV se escriben en el **árbol fuente** del paquete
> (`data/diagnostics/sinusoidal/`, `data/identification/alpha/`), y los nodos
> leen `config/motorXM430W350T_params.yaml` **del árbol fuente al lanzarse**:
> editar ese YAML **no requiere recompilar**.

---

## 3. Paso 1 — Fricción (Fv, Fc): un barrido por articulación

El joint elegido entra en *velocity mode* y recorre magnitudes
0.05…0.50 rad/s en ambos sentidos, rebotando dentro de una banda segura
(±`vel_band` alrededor de su posición de reposo); los demás joints se mantienen
fijos en *position mode*. Duración ≈ 43 s por ensayo (≈ 67 s para J2, que usa
segmentos de 8 s; ver 3.1).

Al terminar el barrido el robot **no se suelta en la pose del ensayo**: frena el
joint en velocidad (0.5 s), pasa todos los joints a *position mode* y regresa
con una quíntica (≥ 3 s, velocidad pico ≤ 0.8 rad/s) a la pose en que estaba
al lanzar el launch; tras 0.5 s de pausa recién corta el torque (≈ 4 s extra,
no se registran en el CSV). Por eso, **lance cada barrido con el robot en una
pose de reposo segura** (p.ej. la pose plegada en que queda al apagarse): es la
pose a la que volverá. El log del nodo muestra `q_inicial` al arrancar y el
error final del retorno al terminar.

**3.1 Ejecutar los 4 barridos** (convención de logs: J1→21, J2→22, J3→23, J4→24):

```bash
ros2 launch open_manipulator_x_torque_control friction_sweep.launch.py friction_joint:=1 log_id:=21

# J2: SIEMPRE con banda angosta y segmentos largos (ver nota abajo)
ros2 launch open_manipulator_x_torque_control friction_sweep.launch.py friction_joint:=2 log_id:=22 \
  vel_band:=0.3 vel_seg_duration:=8.0

ros2 launch open_manipulator_x_torque_control friction_sweep.launch.py friction_joint:=3 log_id:=23
ros2 launch open_manipulator_x_torque_control friction_sweep.launch.py friction_joint:=4 log_id:=24

# dry-run opcional (verifica parámetros sin mover el robot): agregar open_port:=false
ros2 launch open_manipulator_x_torque_control friction_sweep.launch.py friction_joint:=1 log_id:=21 \
  open_port:=false
```

> **J2 siempre con `vel_band:=0.3 vel_seg_duration:=8.0`.** J2 (hombro) carga
> la mayor gravedad del brazo y con la banda por defecto (±0.5 rad, 5 s por
> velocidad) rebota pocas veces a baja velocidad: casi no hay muestras en
> ambos sentidos en las mismas posiciones y el veredicto sale **RE-EJECUTAR**.
> Ocurrió en los dos robots identificados: robot 1, resid 10.6 → 5.84 ticks al
> pasar a banda 0.3; robot 2, resid 10.7 (RE-EJECUTAR) → 1.49 ticks (OK) con
> banda 0.3 y 8 s por velocidad. La banda angosta multiplica las reversiones y
> los segmentos largos dan muestreo suficiente en cada velocidad; `duration_s`
> se recalcula sola (≈ 67 s). J1, J3 y J4 dieron OK con los valores por defecto.

**3.2 Analizar cada log** (el flag `--joint` verifica que el log corresponda a
la articulación esperada).

> **Directorio de trabajo:** los scripts de análisis se ejecutan **desde la
> raíz del paquete**, `~/open_manx_ws/src/open_manipulator_x_torque_control`.
> Todos los comandos `python3 src/Identification/...` y `cp config/...` de este
> protocolo usan rutas relativas a ese directorio (desde otro lugar fallan con
> `can't open file 'src/Identification/...'`). Los `ros2 launch` / `ros2 run`
> funcionan desde cualquier directorio con el workspace *sourceado*.

```bash
cd ~/open_manx_ws/src/open_manipulator_x_torque_control
python3 src/Identification/identify_friction.py 21 --joint 1
python3 src/Identification/identify_friction.py 22 --joint 2
python3 src/Identification/identify_friction.py 23 --joint 3
python3 src/Identification/identify_friction.py 24 --joint 4
```

Cada corrida imprime la curva `I_fric(v)`, el ajuste `Fv·v + Fc·tanh(v/eps)`, la
gráfica en `plots/diagnostics/friction/` y un **VEREDICTO**:

| Veredicto | Significado | Acción |
|---|---|---|
| **OK** | resid ≤ 4 ticks | nada |
| **OK con nota** | el punto de v mínima es fricción de **arranque** (breakaway/Stribeck): real pero fuera del modelo Coulomb cinético → se **excluye** del ajuste oficial (los nodos compensan el régimen cinético) | nada — no repetir |
| **REVISAR** | resid 4–8 ticks, o ajuste bueno pero **barrido truncado** (faltan las velocidades altas para confirmar el plateau) | valor central utilizable; el script imprime el comando exacto para afinar |
| **RE-EJECUTAR** (exit 2) | resid > 8 ticks o < 3 velocidades útiles | repetir con el comando que imprime el script |

Robustez automática del análisis: los puntos con **menos de 2 bins de q
emparejados** se descartan del ajuste (frágiles, p.ej. un 0.20 rad/s con un
solo bin puede salir hasta negativo); el script estima del propio log la
**banda** y la **duración de segmento** realmente usadas, detecta **barridos
truncados** (`duration_s` corto) y adapta el consejo de re-ejecución para no
repetir recetas ya aplicadas.

> **Aviso esperado en J1, J3 y J4: `! sin dato en v = 0.05, 0.15 rad/s`.** Con
> los valores por defecto (banda ±0.5 rad, 5 s por velocidad), a 0.05 rad/s el
> joint recorre solo 0.25 rad en el segmento y nunca llega al borde de la banda
> para invertir; a 0.15 rad/s invierte como mucho una vez. Sin muestras en
> ambos sentidos en las mismas posiciones, esas dos velocidades se descartan.
> Pasó en todos los barridos de referencia (logs 21–24, 31–34 y 41–44). El
> ajuste usa las otras 6 velocidades (0.10–0.50 rad/s), que cubren la meseta
> de Coulomb: con veredicto OK **no hay que repetir el barrido**. J2 sí tiene
> dato a 0.05 rad/s porque corre con banda 0.3 y 8 s (3.1); ese punto suele
> ser el de arranque (OK con nota). Solo repita si faltan más velocidades o si
> el veredicto lo pide.

**3.3 Cuándo y cómo repetir un barrido.** Si J1, J3 o J4 salen dispersos a
baja velocidad (pocos rebotes → poco muestreo bidireccional), aplíqueles la
misma receta que a J2: banda angosta (más reversiones por segmento) y
segmentos más largos, con un `log_id` NUEVO para no pisar el anterior. Por
ejemplo, para J3:

```bash
ros2 launch open_manipulator_x_torque_control friction_sweep.launch.py friction_joint:=3 log_id:=25 \
  vel_band:=0.3 vel_seg_duration:=8.0

cd ~/open_manx_ws/src/open_manipulator_x_torque_control
python3 src/Identification/identify_friction.py 25 --joint 3
```

> `duration_s` es `auto` en el launch: se calcula como
> `t_settle + len(vel_list)·vel_seg_duration + 1 s`, de modo que alargar
> `vel_seg_duration` nunca trunca el barrido. Solo pásela explícita si quiere
> recortarlo a propósito. (Con `ros2 run` directo, el nodo avisa al arrancar si
> `duration_s` no alcanza para todo el `vel_list`.)

La escalera de re-ejecución que aplica el script (y que puede seguir a mano):

1. banda ancha (±0.5) → `vel_band:=0.3` (más reversiones a baja velocidad);
2. banda ya angosta → `vel_seg_duration:=8.0` (más muestreo por velocidad);
3. banda y segmentos ya ajustados y sigue disperso → el problema no es el
   barrido: revisar mecánica (cableado rozando, montaje, backlash), temperatura
   de motores y signos; puede intentarse `vel_seg_duration:=10.0` o aceptar el
   valor central (REVISAR).

J2 ya arranca en el paso 2 (banda 0.3 y 8 s por velocidad desde 3.1): si aun así
sale RE-EJECUTAR, el script recomienda directamente el paso 3.

*Referencia real (robot 2, J2):* con los valores por defecto (log 22) la curva
salió dispersa (0.20 rad/s → −1.6 ticks, de un solo bin) y RE-EJECUTAR; con
banda 0.3 y 8 s por velocidad (log 26) la curva quedó entre 11.5 y 16.5 ticks
con resid 1.49 — veredicto *OK*, Fc ≈ 14.6. El punto de arranque a 0.05 rad/s
(29.9 ticks, un solo bin) quedó fuera del ajuste automáticamente.

Al terminar tendrá 4 fragmentos `data/identification/friction_J<j>_log<id>.yaml`
(anote qué log quedó como definitivo para cada joint).

---

## 4. Paso 2 — α e I_offset: ensayo del lazo de corriente

El nodo `hw_alpha_ident_node` mueve el robot por la trayectoria articular
**act1** del laboratorio de FL (senos de 1 rad/s en J1–J3, probada en el robot
real dentro de límites) con **J4 oscilando ±0.25 rad alrededor de π/4**, en
fase con J1–J3 (en act1 J4 queda fijo y su k no es identificable; en fase, la
pinza queda más lejos del robot que con J4 fijo), con un **PD articular +
compensación de gravedad** del URDF, en modo corriente y con la **conversión
nominal fija en el código** (α = 208.5 ticks/N·m, sin fricción ni offset). Del
YAML de motor solo lee el bloque de hardware (ceros, signos, límites). Por eso:

- **no hay que editar** `config/motorXM430W350T_params.yaml` (da igual qué α
  tenga en ese momento);
- **no se necesita el nodo de Feedback Linearization**;
- el resultado no depende del modelo que use el nodo (no hay circularidad).

Secuencia: rampa de 3 s desde la pose actual → trayectoria hasta `t_run`
(40 s por defecto) → retorno quíntico de 3 s a la pose inicial → 0.5 s de
pausa → torque off (≈ 44 s en total; el retorno no se registra). Lance el
ensayo con el robot en una **pose de reposo segura**: es la pose a la que
volverá.

**4.1 Ejecutar**

```bash
# dry-run opcional: recorre la trayectoria sin hardware y muestra límites y
# corrientes previstas por joint (no escribe CSV)
ros2 launch open_manipulator_x_torque_control alpha_ident.launch.py log_id:=30 open_port:=false

# ensayo real (el robot se mueve)
ros2 launch open_manipulator_x_torque_control alpha_ident.launch.py log_id:=30
```

Argumentos opcionales: `t_run:=60.0` (más datos), `gain_scale:=0.7` (PD más
blando si hay vibración o saturación). El seguimiento será imperfecto: sin
compensación de fricción hay *stick-slip* visible en las inversiones, sobre
todo en J3. **Es esperable y no afecta**: el análisis usa solo la corriente
comandada contra la medida. Si el robot completa el ensayo sin golpes ni
paradas, el log sirve. El nodo se detiene solo si el error de seguimiento
supera 0.5 rad (signos invertidos, colisión) o si la corriente medida supera
`current_measured_peak`.

CSV: `data/identification/alpha/hw_alpha_ident_30.csv`.

**4.2 Analizar** (desde la raíz del paquete, ver 3.2):

```bash
cd ~/open_manx_ws/src/open_manipulator_x_torque_control
python3 src/Identification/identify_alpha.py 30
```

Por joint ajusta `I_meas = k·I_cmd + b` y entrega `α = 208.5/k` e
`I_offset = −b/k`:

- alinea una muestra (los nodos leen el estado antes de escribir la corriente
  del ciclo) y filtra ambas señales con un pasa-bajos de fase cero de 2 Hz;
- un joint usa su propio k si su corriente comandada está bien excitada
  (std ≥ 6 ticks), corr ≥ 0.95 y el **sesgo por cruce por cero** (columna
  `sesgo0%`) es ≤ 2 %. El lazo entrega ~0.4 ticks menos de lo pedido, con el
  signo de la corriente (0.1–0.45 ticks en 23 logs reales): en un joint con
  corriente chica que cambia de signo a menudo, ese desfase baja la pendiente.
  Es el caso de J1 (sin gravedad, solo ±10 ticks de fricción): su k propio sale
  0.92–0.97 aunque el lazo esté bien. Con este ensayo J2 (gravedad), J3
  (fricción) y J4 (fricción y gravedad del efector) usan su propio k; J1 usa
  el k compartido (media de los excitados, mismo motor y firmware) con su
  propio b;
- descarta las muestras con corriente saturada.

| Veredicto | Condición (joints excitados) | Acción |
|---|---|---|
| **OK** | \|k−1\| ≤ 3 % | nada |
| **REVISAR** | \|k−1\| entre 3 y 10 %, \|b\| > 3 ticks, dispersión de k > 3 %, R² < 0.95 o saturación > 2 % | α utilizable; revisar alimentación y temperatura de motores |
| **RE-EJECUTAR** (exit 2) | \|k−1\| > 10 %, sin excitación o log corto | el lazo no sigue la corriente pedida: revisar 12 V bajo carga, `current_limit_register`, modo de operación; repetir |

Resultado de referencia: en 11 logs reales del lazo FL (jul–oct 2026, con
distintos modelos en el YAML) dio k = 0.994–1.003 en J2/J3, es decir
α = 208.0–209.9 ticks/N·m, |I_offset| < 1 tick y R² ≥ 0.99. En los logs de
act2 (J4 se mueve) J4 dio k = 0.98–1.00. Se genera el fragmento
`data/identification/alpha_fit_log30.yaml`.

> Un log del nodo FL (`data/feedback_linearization/real/act1/hw_fl_data_<ID>.csv`) también
> sirve, con cualquier modelo en el YAML: `identify_alpha.py` lo usa si no
> existe un `hw_alpha_ident_<ID>.csv` con el mismo ID (avisa cuando hay más de
> uno).

**Cuándo repetir:** veredicto RE-EJECUTAR, o saturación > 10 % (repetir con
`gain_scale:=0.7`). Use un `log_id` nuevo.

---

## 5. Paso 3 — Ensamblar el YAML final

```bash
cd ~/open_manx_ws/src/open_manipulator_x_torque_control
python3 src/Identification/assemble_motor_params.py \
  --alpha-log 30 --friction-logs 21 22 23 24
```

(`--friction-logs` en orden **J1 J2 J3 J4**, usando el log definitivo de cada
joint — si re-grabó alguno, ponga el nuevo; p.ej. con el J3 re-grabado de 3.3:
`--friction-logs 21 22 25 24`.)

El script valida los veredictos de todos los fragmentos (aborta si alguno quedó
en RE-EJECUTAR), verifica que α esté en `208.5/(1 ± 0.10)`, copia el bloque de
hardware del YAML oficial actual y escribe
`config/motorXM430W350T_params_identified.yaml` con una tabla comparativa
actual↔nuevo. Si el fragmento de α fue generado por la versión anterior de
`identify_alpha.py`, pide volver a analizar el log. Revíselo y actívelo:

```bash
cp config/motorXM430W350T_params_identified.yaml config/motorXM430W350T_params.yaml
```

Luego pase la fricción nueva al URDF estático (`urdf/open_manipulator_x.urdf`):

```bash
python3 src/Identification/update_urdf_friction.py --dry-run   # muestra actual → nuevo
python3 src/Identification/update_urdf_friction.py             # escribe el URDF
```

El script escribe en el `<dynamics>` de joint1..joint4
`damping = motor_Fv/motor_alpha` [N·m·s/rad] y
`friction = motor_Fc/motor_alpha` [N·m] (dominio de torque) y no toca nada
más. Hace falta porque los nodos de simulación `gz_*` cargan ese URDF con
Pinocchio y compensan la fricción con esos valores, mientras que la planta de
Gazebo la toma del xacro, que lee el YAML en cada lanzamiento: si el URDF
queda desactualizado, el controlador simulado compensa una fricción distinta
de la que tiene la planta. Se niega a copiar un YAML con Fv = Fc = 0 (modo
bootstrap) salvo con `--force`. No requiere recompilar (los nodos leen el URDF
del árbol fuente).

---

## 6. Paso 4 — Validación en el robot

El modelo completo se valida con un controlador que lo use. En el curso es el
laboratorio de Feedback Linearization (cada alumno con su
`hw_fl_control_node_student` ya completado); el instructor puede usar el nodo de
referencia:

```bash
ros2 run open_manipulator_x_torque_control hw_fl_control_node \
  --ros-args -p gain_scale:=1.0 -p t_run:=20.0 -p test_num:=31
```

y graficar con `src/Feedback Linearization/MATLAB/plots_FL_control.m`
(`test_num = 31`). Referencia de un robot sano con este modelo (lazo a 200 Hz):
errores RMS articulares del orden de **0.005–0.03 rad** por
joint tras la rampa, sin saturación de torque y sin stick-slip visible en las
inversiones. Si hay stick-slip marcado o saturación, revise la tabla
comparativa del ensamblado y la sección de problemas.

Opcionalmente, `identify_alpha.py 31` sobre ese log debe dar el mismo k
(±1 %) que el Paso 2: el método no depende del modelo que use el nodo, así que
sirve como verificación de reproducibilidad (también vale repetir
`alpha_ident.launch.py` con otro `log_id`).

---

## 7. Solución de problemas

| Síntoma | Causa probable | Acción |
|---|---|---|
| identify_alpha: k fuera de 1 ± 10 % (RE-EJECUTAR) | el lazo de corriente no sigue lo pedido: caída de los 12 V bajo carga, `current_limit_register` bajo, motor caliente | revisar fuente y cableado, dejar enfriar los motores y repetir el Paso 2 |
| identify_alpha: "ningún joint tiene corriente comandada" | se pasó un log de barrido (velocity/position) | analizar el log de `alpha_ident.launch.py` (Paso 2) |
| assemble: "fragmento de la versión ANTERIOR de identify_alpha.py" | fragmento generado con el método antiguo (regresión contra el torque) | volver a correr `identify_alpha.py <ID>` sobre el mismo log |
| Parada "Error de seguimiento > 0.5 rad" en el ensayo de α | `current_sign`/`encoder_sign` invertidos, colisión o pose inicial trabada | verificar signos con `hw_torque_probe_node`, despejar el robot y repetir |
| `No se pudo abrir /dev/ttyUSB0` | otro proceso usa el puerto / permisos | cerrar otros nodos (ros2_control, otro launch); grupo dialout |
| Parada `Corriente insegura J*` | colisión, trayectoria fuera de rango, cableado | despejar el robot; verificar `joint_zero_tick` (¿robot ensamblado en la pose cero?) |
| Warnings de overrun del lazo (>Ts) | latency_timer ≠ 1 ms | requisito 2.3; o bajar `loop_rate_hz` a 100 |
| J2 sale RE-EJECUTAR en fricción | se lanzó con la banda por defecto (±0.5 rad, 5 s por velocidad): rebote pobre a baja velocidad | lanzar J2 siempre con `vel_band:=0.3 vel_seg_duration:=8.0` (3.1) |
| Lancé un launch para verificar y el robot se movió | los flags de hardware son `true` por defecto | dry-run con `open_port:=false` |
| Barrido sin velocidades altas ("BARRIDO TRUNCADO") | `duration_s` explícita menor que `t_settle + n_vel·vel_seg_duration` | usar `duration_s` auto del launch (o pasarla ≥ la suma); el nodo también lo advierte al arrancar |
| Dispersión persiste con banda 0.3 y segmentos 8 s | causa mecánica, no de configuración | revisar cableado/montaje/backlash, temperatura; `vel_seg_duration:=10.0` o aceptar valor central |
| Puntos `I_fric` negativos | `current_sign`/`encoder_sign` invertidos | corregir signos en el YAML y repetir el barrido |
| El joint en velocity no se mueve / se sale de banda | `A_pos` fuera de límites o banda excesiva | revisar `hw_friction_sweep_params.yaml` (A_pos, vel_band vs joint_lower/upper) |
| CSV no aparece | se busca en el árbol instalado | los datos van al árbol FUENTE: `data/…` del repositorio |
| `can't open file '…/src/Identification/…'` | script lanzado fuera de la raíz del paquete | `cd ~/open_manx_ws/src/open_manipulator_x_torque_control` (ver 3.2) |
| Aviso "El robot quedó a X rad de q_inicial" al final del barrido | el retorno chocó con un obstáculo o la pose inicial estaba forzada contra un tope | despejar el robot y lanzar el barrido desde una pose de reposo libre |

---

## 8. Archivos del pipeline (referencia)

```
src/Identification/
├── Ident_OpenManX_XM430W350T_procedure.md   ← este protocolo
├── hw_sinusoidal_torque_node.cpp    nodo de excitación (current/position/velocity)
├── hw_alpha_ident_node.cpp          ensayo de α: PD + G(q) con conversión nominal fija
├── identify_friction.py             Fv, Fc   (barrido de velocidad + reversión)
├── identify_alpha.py                alpha, I_offset (ganancia del lazo de corriente)
├── assemble_motor_params.py         ensambla el YAML final
├── update_urdf_friction.py          copia Fv/α y Fc/α del YAML al URDF estático
├── identify_motor_model.py          referencia modo posición (NO usar su α)
└── MATLAB/sinusoidal_torque_analysis.m

launch/friction_sweep.launch.py      barrido de fricción por articulación
launch/alpha_ident.launch.py         ensayo de α (Paso 2)
config/hw_friction_sweep_params.yaml parámetros del barrido (vel_list, banda…)
config/hw_sinusoidal_torque_params.yaml  parámetros generales del nodo de excitación
config/motorXM430W350T_params.yaml   ← DESTINO FINAL del modelo
urdf/open_manipulator_x.urdf         <dynamics> de joint1..4 (update_urdf_friction.py)

data/diagnostics/sinusoidal/hw_sin_torque_<id>.csv   logs de barridos
data/identification/alpha/hw_alpha_ident_<id>.csv    logs del ensayo de α
data/identification/*.yaml                           fragmentos (entrada del ensamblador)
config/motorXM430W350T_params_identified.yaml        YAML ensamblado (revisar y copiar)
```
