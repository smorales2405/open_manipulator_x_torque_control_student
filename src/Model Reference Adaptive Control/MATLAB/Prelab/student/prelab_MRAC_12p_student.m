%% prelab_MRAC_12p_student.m
% Prelab — Control Adaptativo (MRAC / Slotine-Li) articular de 12 parametros
% OpenMANIPULATOR-X — simulacion MATLAB equivalente al nodo gz_MRAC_joint_12p_node.cpp
% [Archivo base para estudiantes]
%
% Replica la estructura del nodo (misma trayectoria, ganancias, tasas de
% adaptacion, lazo a 200 Hz y saturacion), sustituyendo Gazebo por una planta
% integrada con OMDyn (dinamica rigida generada del URDF) mas friccion
% articular viscosa + Coulomb.
%
%   Parametros adaptados (12):
%     a_hat = [alpha1..alpha3 | dm, dmcx, dmcy, dmcz | Fv1 | Fc1..Fc4]'
%       alpha_k    : escala de los parametros inerciales del cuerpo movido por
%                    joint k (link2..link4), nominal 1
%       theta_load : exceso de masa [kg] y de primer momento [kg·m] del ultimo
%                    cuerpo (link5 y lo que sujete el gripper), nominal 0
%       Fv1        : friccion viscosa de joint1 [N·m·s/rad]
%       Fc_j       : friccion de Coulomb de joint j [N·m]
%
%   Planta simulada:
%     ms*M(q)*ddq + ms*phib(q,dq) + ds*Fv.*dq + fs*Fc.*tanh(dq/eps_plant) = tau_sat
%   integrada con RK4 a Ts/N_SUB con ZOH del torque. Los controladores usan
%   siempre el modelo nominal (ms = ds = fs = 1).
%
%   Actividades (escalas 1.0/1.5/1.5 en ambas: friccion viscosa y de Coulomb
%   50 % mayores que las nominales que conoce el controlador):
%     ACTIVIDAD = 1 : MRAC fijo vs MRAC adaptativo
%     ACTIVIDAD = 2 : SMC con integral vs MRAC adaptativo
%
%   Referencia: sinusoide articular (w = 1 rad/s). Sin
%   transicion: la planta parte en q_des(0) y en reposo, y las metricas
%   excluyen t < T_REG (fase de alcance).
%
%   Al arrancar se verifica la SECCION 2 (torque nominal y regresor) contra la
%   dinamica rigida de los cuerpos con parametros perturbados.
%
% Visualizacion 3D del MRAC adaptativo (cadena del URDF + STLs de meshes/):
%   ANIM_MODE = 'post' — simula todo el horizonte y luego anima con
%               dt = 1/200 s por frame en tiempo real (salta frames si el
%               render no alcanza 200 fps).
%   ANIM_MODE = 'live' — renderiza durante la simulacion del MRAC adaptativo,
%               como si fuera Gazebo (~50 fps, al ritmo del reloj de pared).
%   ANIM_MODE = 'off'  — sin animacion.
%
% Figuras comparativas: posiciones, errores, torques y superficies de las dos
% leyes de la actividad y parametros estimados del MRAC adaptativo. Metricas
% en consola.
%
% CARPETA AUTOCONTENIDA (portable): copiar esta carpeta completa a cualquier
% maquina y ejecutar el script desde ahi. Contenido:
%   prelab_MRAC_12p_student.m   este script
%   OMDyn.mexa64           MEX de OMDyn precompilado para Linux
%   OMDyn.mexw64           MEX de OMDyn precompilado para Windows
%   meshes/*.stl           mallas del robot para la vista 3D
%
% ──────────────────────────────────────────────────────────────────────────
% SECCIONES A COMPLETAR:
%   [1] Trayectoria articular de referencia  ->  trayectoria_deseada()
%   [2] Modelo nominal y regresor            ->  tau_nominal(), regresor()
%   [3] Ganancias MRAC y tasas Gamma         ->  LAMBDA_Q, K_V, K_S, PHI, GAMMA_*
%       Ganancias del SMC con integral       ->  LAMBDA_SMC, K_V_SMC, K_S_SMC,
%                                                K_I_SMC, PHI_SMC (Actividad 2)
%   [4] Ley de control y adaptacion MRAC     ->  lazo de control
% Al iniciar, el script verifica la SECCION 2 y se detiene con un informe por
% bloque si algo no coincide.

clear; clc; close all;

%% ── Configuracion ────────────────────────────────────────────────────────────
ACTIVIDAD   = 1;       % 1: MRAC fijo vs MRAC adaptativo
                       % 2: SMC con integral vs MRAC adaptativo
T_SIM       = 25.0;    % [s] duracion de la simulacion (t_run del nodo)
ANIM_MODE   = 'post';  % 'post' | 'live' | 'off'  (ver cabecera)

EXPORT_FIGS = false;   % true = guardar PNG (300 dpi) y EPS vectorial (600 dpi) en
                       %        plots/act<ACTIVIDAD>/ junto al script

% ── Escalas de la planta (como en config/control/sim_init_config.yaml) ────────
% Solo afectan a la planta simulada; los controladores usan siempre el modelo
% nominal. Las dos actividades usan 1.0 / 1.5 / 1.5.
mass_inertia_scale = 1.0;   % factor sobre masas e inercias de los eslabones
damping_scale      = 1.5;   % factor sobre la friccion viscosa Fv
friction_scale     = 1.5;   % factor sobre la friccion de Coulomb Fc

% ═══════════════════════════════════════════════════════════════════════════
%  SECCION 3 — GANANCIAS MRAC Y TASAS DE ADAPTACION  (COMPLETAR)
%  Indice de cada elemento: [joint1, joint2, joint3, joint4]
%
%  Rangos recomendados (lazo discreto a 200 Hz, Ts = 5 ms):
%    LAMBDA_Q : 5  – 30    [1/s]     (ancho de banda de la superficie)
%    K_V      : 5  – 50              (alcance exponencial; proporcional a s)
%    K_S      : 1  – 40    [rad/s^2] (ganancia de conmutacion)
%    PHI      : 0.2 – 1.0  [rad/s]   (capa limite de sat, comun a las 4 articulaciones)
%
%  Tasas de adaptacion Gamma, una por bloque de parametros (todas > 0):
%    GAMMA_ALPHA : 0.1   – 0.5     (alpha1..alpha3)
%    GAMMA_DM    : 0.02  – 0.1     (dm, masa de la carga)
%    GAMMA_MC    : 0.001 – 0.005   (dmcx, dmcy, dmcz, primer momento de la carga)
%    GAMMA_FV    : 0.5   – 3.0     (Fv1)
%    GAMMA_FC    : 0.5   – 2.0     (Fc1..Fc4)
% ═══════════════════════════════════════════════════════════════════════════
LAMBDA_Q = [0.0; 0.0; 0.0; 0.0];   % COMPLETAR
K_V      = [0.0; 0.0; 0.0; 0.0];   % COMPLETAR
K_S      = [0.0; 0.0; 0.0; 0.0];   % COMPLETAR
PHI      = 0.0;                    % COMPLETAR  [rad/s]

GAMMA_ALPHA = 0.0;   % COMPLETAR
GAMMA_DM    = 0.0;   % COMPLETAR
GAMMA_MC    = 0.0;   % COMPLETAR
GAMMA_FV    = 0.0;   % COMPLETAR
GAMMA_FC    = 0.0;   % COMPLETAR
% ═══════════════════════════════════════════════════════════════════════════

% ═══════════════════════════════════════════════════════════════════════════
%  GANANCIAS DEL SMC CON INTEGRAL (Actividad 2)  (COMPLETAR)
%  Las de gz_SMC_joint_node_student.cpp del Laboratorio 6, con integral:=true.
%  Indice de cada elemento: [joint1, joint2, joint3, joint4]
%
%  Rangos recomendados:
%    LAMBDA_SMC : 5  – 30    [1/s]
%    K_V_SMC    : 5  – 50
%    K_S_SMC    : 1  – 40    [rad/s^2]
%    K_I_SMC    : 50 – 800   [1/s^2]
%    PHI_SMC    : 0.2 – 1.0  [rad/s]
% ═══════════════════════════════════════════════════════════════════════════
LAMBDA_SMC = [0.0; 0.0; 0.0; 0.0];   % COMPLETAR
K_V_SMC    = [0.0; 0.0; 0.0; 0.0];   % COMPLETAR
K_S_SMC    = [0.0; 0.0; 0.0; 0.0];   % COMPLETAR
K_I_SMC    = [0.0; 0.0; 0.0; 0.0];   % COMPLETAR
PHI_SMC    = 0.0;                    % COMPLETAR  [rad/s]
% ═══════════════════════════════════════════════════════════════════════════

% ── Parametros fijos (no modificar) ──────────────────────────────────────────
TAU_MAX = 1.2;         % [N·m] limite de torque (tau_max del robot real)
Ts      = 1/200;       % [s] periodo del lazo de control (200 Hz, como el nodo)
T_REG   = 1.0;         % [s] ventana excluida de las metricas: fase de alcance
                       %     (el robot parte en reposo y dq_des(0) ~= 0)

% Friccion articular nominal. La planta la simula escalada
% y los controladores la usan sin escalar: es el prior del MRAC y
% el feedforward del SMC.
Fv = 0.02 * ones(4,1);   % Damping Friction [N·m·s/rad] igual en las 4 articulaciones
Fc = 0.05 * ones(4,1);   % Coulomb Friction [N·m] igual en las 4 articulaciones
F_EPS     = 0.05;   % [rad/s] suavizado del tanh de Coulomb en los controladores
eps_plant = 0.01;   % [rad/s] suavizado del sign() de Coulomb en la planta
N_SUB     = 100;    % subpasos RK4 por tick (h = Ts/N_SUB = 0.05 ms): el Coulomb
                    % de la planta es rigido cerca de dq = 0 (ver prelab_SMC.m)

% Fuga sigma hacia el prior [1/s] y cotas de los parametros, por bloque
% [alpha1..3, dm, dmcx, dmcy, dmcz, Fv1, Fc1..4] 
SIGMA_LEAK = [0.1; 0.1; 0.1;   0.02; 0.02; 0.02; 0.02;   0.1;   0.1; 0.1; 0.1; 0.1];
A_MIN      = [0.80; 0.80; 0.80;   -0.02; -0.005; -0.010; -0.005;   0.0;   0.0; 0.0; 0.0; 0.0];
A_MAX      = [1.25; 1.25; 1.25;    0.20;  0.035;  0.010;  0.020;   0.15;  0.30; 0.30; 0.30; 0.30];

% Tope anti-windup del termino integral del SMC |z_i| <= Z_MAX_i [rad/s^2]
Z_MAX = [20.0; 100.0; 150.0; 600.0];

% Limites articulares del URDF (solo verificacion post-simulacion)
Q_LOWER = [-2.3562; -1.9199; -1.9199; -1.8];
Q_UPPER = [ 2.3562;  1.7453;  1.5708;  2.1];

% Tasas de adaptacion por parametro, en el orden de a_hat
GAMMA = [GAMMA_ALPHA*ones(3,1); GAMMA_DM; GAMMA_MC*ones(3,1); GAMMA_FV; GAMMA_FC*ones(4,1)];

% Prior de parametros (= a_hat(0)): alphas en 1, carga en 0 y friccion nominal
a_prior = [ones(3,1); zeros(4,1); Fv(1); Fc];

% Modelo nominal para la SECCION 2: friccion nominal y parametros dinamicos
% de los cuerpos movidos por joint1..joint4 (URDF)
P.FV0    = Fv;
P.FC0    = Fc;
P.PI_NOM = pi_cuerpos();

%% ── Rutas y dependencias (autocontenidas en esta carpeta) ────────────────────
this_dir = fileparts(mfilename('fullpath'));
mesh_dir = fullfile(this_dir, 'meshes');
addpath(this_dir);   % OMDyn resoluble aunque se ejecute desde otro cwd

assert(~isempty(which('OMDyn')), ...
    'OMDyn.%s no encontrado junto al script para esta plataforma.', mexext);
assert(isfolder(mesh_dir), 'No se encontro la carpeta meshes/ junto al script.');
assert(any(ACTIVIDAD == [1 2]), 'ACTIVIDAD debe ser 1 o 2.');
assert(any(strcmp(ANIM_MODE, {'post', 'live', 'off'})), ...
    'ANIM_MODE debe ser ''post'', ''live'' u ''off''.');

% PHI y Gamma se completan en la SECCION 3: sat(s/PHI) necesita PHI > 0 y la
% adaptacion necesita Gamma > 0
assert(PHI > 0, 'PHI = %.3f: completar PHI > 0 en la SECCION 3.', PHI);
assert(all(GAMMA > 0), ['Gamma sin completar: GAMMA_ALPHA, GAMMA_DM, GAMMA_MC, ' ...
       'GAMMA_FV y GAMMA_FC deben ser > 0 en la SECCION 3.']);
if ACTIVIDAD == 2
    assert(PHI_SMC > 0, 'PHI_SMC = %.3f: completar PHI_SMC > 0.', PHI_SMC);
end

fprintf('Modelo OMDyn: %s\n', which('OMDyn'));
fprintf('Actividad %d  |  escalas de la planta: masa=%.2f  damping=%.2f  friccion=%.2f\n', ...
        ACTIVIDAD, mass_inertia_scale, damping_scale, friction_scale);
fprintf('MRAC: Lambda=[%.1f %.1f %.1f %.1f]  Kv=[%.1f %.1f %.1f %.1f]  Ks=[%.1f %.1f %.1f %.1f]  PHI=%.2f\n', ...
        LAMBDA_Q, K_V, K_S, PHI);
fprintf('      Gamma: alpha=%.3f  dm=%.3f  mc=%.4f  Fv1=%.2f  Fc=%.2f\n', ...
        GAMMA_ALPHA, GAMMA_DM, GAMMA_MC, GAMMA_FV, GAMMA_FC);
if ACTIVIDAD == 2
    fprintf(['SMC:  Lambda=[%.1f %.1f %.1f %.1f]  Kv=[%.1f %.1f %.1f %.1f]  Ks=[%.1f %.1f %.1f %.1f]  ' ...
             'Ki=[%.0f %.0f %.0f %.0f]  PHI=%.2f\n'], LAMBDA_SMC, K_V_SMC, K_S_SMC, K_I_SMC, PHI_SMC);
end
fprintf('Friccion nominal: Fv=[%.4f %.4f %.4f %.4f]  Fc=[%.4f %.4f %.4f %.4f]\n', Fv, Fc);
fprintf('Tiempo de simulacion: %.1f s  |  lazo: %.0f Hz  |  animacion: %s\n\n', ...
        T_SIM, 1/Ts, ANIM_MODE);

%% ── Referencia precomputada (SECCION 1) ──────────────────────────────────────
% Nsim = numero de ticks de control (200 Hz); se registra un punto extra al
% final (estado tras el ultimo tick) para que t_log(end) = T_SIM exacto.
Nsim  = round(T_SIM / Ts);
t_log = (0:Nsim)' * Ts;

QD = zeros(Nsim+1, 4); DQD = zeros(Nsim+1, 4); DDQD = zeros(Nsim+1, 4);
for k = 1:Nsim+1
    [qd, dqd, ddqd] = trayectoria_deseada(t_log(k));
    QD(k,:) = qd'; DQD(k,:) = dqd'; DDQD(k,:) = ddqd';
end

% Condicion inicial de la planta: en la posicion inicial de la trayectoria y
% en reposo (sin transicion — ver cabecera)
q0  = QD(1,:)';
dq0 = zeros(4,1);
fprintf('Pose inicial = q_des(0) = [%.3f %.3f %.3f %.3f] rad (en reposo)\n\n', q0);

% Trayectoria cartesiana deseada del efector (para la vista 3D)
Y_DES = zeros(Nsim+1, 4);
for k = 1:Nsim+1
    Y_DES(k,:) = fkin_local(QD(k,:)')';
end

%% ── Autoverificacion de la SECCION 2 (torque nominal y regresor) ─────────────
autoverificacion(P);

%% ── Vista 3D (solo modo 'live': se construye antes del lazo) ─────────────────
H = [];
if strcmp(ANIM_MODE, 'live')
    H = build_robot3d(mesh_dir, q0, Y_DES);
    ANIM_DECIM = 4;            % render cada 4 ticks (~50 fps)
end

%% ── Lazo de control a 200 Hz (tick del nodo) ─────────────────────────────────
% Se simula la misma planta con cada ley de la actividad; LOG guarda los
% registros de cada una.
if ACTIVIDAD == 1
    LEYES = {'MRAC_fijo', 'MRAC'};
else
    LEYES = {'SMC_I', 'MRAC'};
end
NOMBRE = struct('MRAC_fijo', 'MRAC fijo', 'MRAC', 'MRAC adaptativo', ...
                'SMC_I', 'SMC con integral');
LOG = struct();

% Parametros de la planta: nominales multiplicados por las escalas
Fv_plant = damping_scale  * Fv;
Fc_plant = friction_scale * Fc;

for i_ley = 1:numel(LEYES)
    ley      = LEYES{i_ley};
    adaptive = strcmp(ley, 'MRAC');
    live     = strcmp(ANIM_MODE, 'live') && adaptive;

    Q  = zeros(Nsim+1, 4); DQ = zeros(Nsim+1, 4); TAU = zeros(Nsim+1, 4);
    S  = zeros(Nsim+1, 4); AH = zeros(Nsim+1, 12);

    x        = [q0; dq0];
    a_hat    = a_prior;       % estimacion de los parametros (MRAC)
    z        = zeros(4,1);    % termino integral (SMC)
    sat_prev = false;         % el tick anterior saturo (anti-windup del SMC)
    t0_wall  = tic;
    for k = 1:Nsim
        % ── Leer estado (en el nodo: /joint_states) ───────────────────────
        q  = x(1:4);
        dq = x(5:8);

        % ── Referencia del tick ───────────────────────────────────────────
        qd = QD(k,:)'; dqd = DQD(k,:)'; ddqd = DDQD(k,:)';

        if startsWith(ley, 'MRAC')
            % ═══════════════════════════════════════════════════════════════
            %  SECCION 4 — Ley de control y adaptacion MRAC  (COMPLETAR)
            %  Disponible: q, dq, qd, dqd, ddqd (4x1), a_hat, a_prior (12x1),
            %              LAMBDA_Q, K_V, K_S, PHI, GAMMA, SIGMA_LEAK
            % ═══════════════════════════════════════════════════════════════
            % 4.1 Errores articulares
            e_q  = zeros(4,1);   % COMPLETAR
            e_dq = zeros(4,1);   % COMPLETAR

            % 4.2 Superficie deslizante y referencias de Slotine-Li
            s_q  = zeros(4,1);   % COMPLETAR
            dqr  = zeros(4,1);   % COMPLETAR
            ddqr = zeros(4,1);   % COMPLETAR

            % M, torque nominal y regresor de la SECCION 2 (no modificar)
            th      = tanh(dqd / F_EPS);
            M       = OMDyn(q, dq);
            tau_nom = tau_nominal(q, dq, dqr, ddqr, th, P);
            Y       = regresor(q, dq, dqr, ddqr, th, P);

            % 4.3 Funcion de conmutacion: rho = rho_vec(s_q, ...)
            rho = zeros(4,1);    % COMPLETAR

            % 4.4 Torque
            tau = zeros(4,1);    % COMPLETAR

            % 4.5 Ley de adaptacion (derivada de a_hat, 12x1)
            a_dot = zeros(12,1); % COMPLETAR
            % ═══════════════════════════════════════════════════════════════

            tau_sat = min(max(tau, -TAU_MAX), TAU_MAX);
            AH(k,:) = a_hat';

            % Integracion de a_hat (como en el nodo): Euler, proyeccion a
            % [A_MIN, A_MAX] y congelada si algun torque satura
            if adaptive && ~any(abs(tau) > TAU_MAX)
                a_hat = min(max(a_hat + Ts*a_dot, A_MIN), A_MAX);
            end
        else
            % ── SMC con integral
            [M, phib] = OMDyn(q, dq);
            e_q  = q  - qd;
            e_dq = dq - dqd;
            s_q  = e_dq + LAMBDA_SMC.*e_q;
            rho  = rho_vec(s_q, PHI_SMC);
            if ~sat_prev   % congelado si el tick anterior saturo
                z = min(max(z + Ts*K_I_SMC.*s_q, -Z_MAX), Z_MAX);
            end
            v_q = ddqd - LAMBDA_SMC.*e_dq - K_V_SMC.*s_q - K_S_SMC.*rho - z;
            tau = M*v_q + phib + Fv.*dq + Fc.*tanh(dqd / F_EPS);   % friccion despues de la ley
            tau_sat  = min(max(tau, -TAU_MAX), TAU_MAX);
            sat_prev = any(abs(tau) >= TAU_MAX);
        end

        % ── Registro (columnas del CSV del nodo) ──────────────────────────
        Q(k,:) = q'; DQ(k,:) = dq'; S(k,:) = s_q'; TAU(k,:) = tau_sat';

        % ── Planta (el papel de Gazebo): RK4 con ZOH del torque ──────────
        h = Ts / N_SUB;
        for i_sub = 1:N_SUB
            x = rk4_step(x, tau_sat, h, Fv_plant, Fc_plant, eps_plant, mass_inertia_scale);
        end

        % ── Animacion en vivo + ritmo de tiempo real (solo 'live') ───────
        if live
            if mod(k-1, ANIM_DECIM) == 0
                trace_add(H, q);
                if toc(t0_wall) <= t_log(k) + Ts
                    update_robot3d(H, q, t_log(k));
                    drawnow;
                end
            end
            sobra = t_log(k) + Ts - toc(t0_wall);
            if sobra > 0, pause(sobra); end
        end

        if mod(k, round(1/Ts)) == 0
            if startsWith(ley, 'MRAC')
                fprintf('[%-15s] t=%5.2f s  |e|=%.5f rad  |s|=%.4f  Fv1=%.4f  Fc=[%.4f %.4f %.4f %.4f]\n', ...
                        NOMBRE.(ley), t_log(k), norm(e_q), norm(s_q), a_hat(8), a_hat(9:12));
            else
                fprintf('[%-15s] t=%5.2f s  |e|=%.5f rad  |s|=%.4f  z=[%+.2f %+.2f %+.2f %+.2f]\n', ...
                        NOMBRE.(ley), t_log(k), norm(e_q), norm(s_q), z);
            end
        end
    end

    % Punto final de registro: estado de la planta tras el ultimo tick (t=T_SIM
    % exacto); torque, parametros y z se mantienen (ZOH)
    Q(Nsim+1,:)   = x(1:4)';
    DQ(Nsim+1,:)  = x(5:8)';
    TAU(Nsim+1,:) = TAU(Nsim,:);
    AH(Nsim+1,:)  = a_hat';
    if strcmp(ley, 'SMC_I')
        lam = LAMBDA_SMC;
    else
        lam = LAMBDA_Q;
    end
    S(Nsim+1,:) = ((x(5:8) - DQD(Nsim+1,:)') + lam.*(x(1:4) - QD(Nsim+1,:)'))';

    fprintf('Simulacion %s completada (%.1f s).\n\n', NOMBRE.(ley), T_SIM);

    % Verificacion de limites articulares del URDF
    for i = 1:4
        if any(Q(:,i) < Q_LOWER(i)) || any(Q(:,i) > Q_UPPER(i))
            warning('%s: q%d salio de los limites del URDF [%.3f, %.3f] rad.', ...
                    NOMBRE.(ley), i, Q_LOWER(i), Q_UPPER(i));
        end
    end

    LOG.(ley) = struct('Q', Q, 'DQ', DQ, 'TAU', TAU, 'S', S, 'AH', AH);
end

%% ── Metricas (sin la fase de alcance) ────────────────────────────────────────
m_reg = t_log >= T_REG;
fprintf('%s\n', repmat('═', 1, 94));
fprintf(' Actividad %d  [Prelab MRAC 12p | escalas %.2f/%.2f/%.2f]  (t >= %.1f s)\n', ...
        ACTIVIDAD, mass_inertia_scale, damping_scale, friction_scale, T_REG);
fprintf('%s\n', repmat('═', 1, 94));
for i_ley = 1:numel(LEYES)
    L = LOG.(LEYES{i_ley});
    fprintf('\n  %s\n', NOMBRE.(LEYES{i_ley}));
    fprintf('  %-6s  %-12s  %-12s  %-14s  %-14s  %-8s  %-10s\n', ...
            'Joint', 'e_max[rad]', 'e_RMS[rad]', 'max|tau|[N·m]', 'tau_RMS[N·m]', 'Sat[%]', 'TV(tau)');
    fprintf('  %s\n', repmat('-', 1, 86));
    for i = 1:4
        e_i   = L.Q(m_reg, i) - QD(m_reg, i);
        tau_i = L.TAU(m_reg, i);
        fprintf('  q%-5d  %12.5f  %12.5f  %14.4f  %14.4f  %8.2f  %10.4f\n', i, ...
                max(abs(e_i)), sqrt(mean(e_i.^2)), max(abs(tau_i)), sqrt(mean(tau_i.^2)), ...
                100 * mean(abs(tau_i) >= 0.99*TAU_MAX), sum(abs(diff(tau_i))));
    end
end
fprintf('\n  Resumen e_RMS [mrad]\n');
fprintf('  %-20s %9s %9s %9s %9s\n', 'Ley', 'q1', 'q2', 'q3', 'q4');
for i_ley = 1:numel(LEYES)
    L = LOG.(LEYES{i_ley});
    fprintf('  %-20s %9.2f %9.2f %9.2f %9.2f\n', NOMBRE.(LEYES{i_ley}), ...
            1e3 * sqrt(mean((L.Q(m_reg,:) - QD(m_reg,:)).^2, 1)));
end

% Valores verdaderos de los parametros en la planta simulada
ALPHA_TRUE = mass_inertia_scale * ones(3,1);
LOAD_TRUE  = (mass_inertia_scale - 1) * P.PI_NOM(1:4, 4);   % link5 escalado
FV1_TRUE   = damping_scale * Fv(1);
FC_TRUE    = friction_scale * Fc;
ah = LOG.MRAC.AH(end,:)';
fprintf('\n  Parametros finales del MRAC adaptativo (verdadero entre parentesis):\n');
fprintf('    alpha = [%.3f %.3f %.3f]            (%.2f)\n', ah(1:3), mass_inertia_scale);
fprintf('    carga = dm=%.4f kg  dmc=[%.4f %.4f %.4f] kg·m   (dm=%.4f, dmc=[%.4f %.4f %.4f])\n', ...
        ah(4:7), LOAD_TRUE);
fprintf('    Fv1   = %.4f N·m·s/rad                (%.4f)\n', ah(8), FV1_TRUE);
fprintf('    Fc    = [%.4f %.4f %.4f %.4f] N·m    (%.4f %.4f %.4f %.4f)\n', ah(9:12), FC_TRUE);
fprintf('%s\n\n', repmat('═', 1, 94));

%% ── Animacion 3D posterior del MRAC adaptativo (dt = 1/200 s por frame) ──────
if strcmp(ANIM_MODE, 'post')
    QA = LOG.MRAC.Q;
    H = build_robot3d(mesh_dir, q0, Y_DES);
    drawnow;
    t0_wall = tic;
    k = 1;
    while k <= Nsim
        update_robot3d(H, QA(k,:)', t_log(k));
        drawnow;
        % siguiente frame segun el reloj: mantiene dt = 1/200 s de movimiento
        % en tiempo real aunque el render no alcance 200 fps (salta frames)
        k_next = max(k + 1, 1 + floor(toc(t0_wall) / Ts));
        for k_skip = k:min(k_next - 1, Nsim+1)
            trace_add(H, QA(k_skip,:)');
        end
        k = k_next;
    end
    update_robot3d(H, QA(end,:)', t_log(end));
    trace_add(H, QA(end,:)');
    drawnow;
end

%% ── Figuras comparativas ─────────────────────────────────────────────────────
lw         = 1.5;
fs         = 11;
fs_title   = 12;
color_ref  = [0.40 0.40 0.40];                       % gris — referencia
colors     = [0.0000 0.4470 0.7410;                  % azul    — primera ley
              0.8500 0.3250 0.0980];                 % naranja — MRAC adaptativo
c_par      = lines(4);
jointNames = {'Articulacion 1', 'Articulacion 2', 'Articulacion 3', 'Articulacion 4'};
xlims      = [t_log(1), t_log(end)];
nombres    = cellfun(@(l) NOMBRE.(l), LEYES, 'UniformOutput', false);
mode_label = sprintf('Prelab MRAC | Actividad %d | escalas %.1f/%.1f/%.1f', ACTIVIDAD, ...
                     mass_inertia_scale, damping_scale, friction_scale);

% ── Figura 1 — Posiciones articulares ────────────────────────────────────────
figure(1); clf;
set(gcf, 'Color', 'w', 'Position', [100 100 1100 700]);
tl = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for i = 1:4
    ax = nexttile(tl);
    h_ref = plot(t_log, QD(:,i), '--', 'Color', color_ref, 'LineWidth', 1.2); hold on;
    h = gobjects(1, numel(LEYES));
    for j = 1:numel(LEYES)
        h(j) = plot(t_log, LOG.(LEYES{j}).Q(:,i), '-', 'Color', colors(j,:), 'LineWidth', lw);
    end
    if i == 1, ax1 = ax; h1 = [h_ref, h]; end
    xlabel('Tiempo [s]', 'FontSize', fs);
    ylabel(sprintf('$q_%d$ [rad]', i), 'Interpreter', 'latex', 'FontSize', fs);
    title(jointNames{i}, 'FontSize', fs_title);
    grid on; box on; set(gca, 'FontSize', fs); xlim(xlims);
end
lgd = legend(ax1, h1, [{'Referencia'}, nombres], 'Orientation', 'horizontal', 'FontSize', fs);
lgd.Layout.Tile = 'north';
title(tl, sprintf('[%s] Seguimiento de posiciones articulares', mode_label), ...
      'FontSize', 14, 'FontWeight', 'bold');

% ── Figura 2 — Errores de seguimiento articular ──────────────────────────────
figure(2); clf;
set(gcf, 'Color', 'w', 'Position', [120 120 1100 600]);
tl = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for i = 1:4
    ax = nexttile(tl);
    h = gobjects(1, numel(LEYES));
    for j = 1:numel(LEYES)
        h(j) = plot(t_log, LOG.(LEYES{j}).Q(:,i) - QD(:,i), '-', 'Color', colors(j,:), ...
                    'LineWidth', lw); hold on;
    end
    yline(0, ':', 'LineWidth', 0.8, 'HandleVisibility', 'off');
    if i == 1, ax1 = ax; h1 = h; end
    xlabel('Tiempo [s]', 'FontSize', fs);
    ylabel(sprintf('$e_{q_%d}$ [rad]', i), 'Interpreter', 'latex', 'FontSize', fs);
    title(jointNames{i}, 'FontSize', fs_title);
    grid on; box on; set(gca, 'FontSize', fs); xlim(xlims);
end
lgd = legend(ax1, h1, nombres, 'Orientation', 'horizontal', 'FontSize', fs);
lgd.Layout.Tile = 'north';
title(tl, sprintf('[%s] Errores de seguimiento articular', mode_label), ...
      'FontSize', 14, 'FontWeight', 'bold');

% ── Figura 3 — Torques de control ────────────────────────────────────────────
figure(3); clf;
set(gcf, 'Color', 'w', 'Position', [140 140 1100 700]);
tl = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for i = 1:4
    ax = nexttile(tl);
    h = gobjects(1, numel(LEYES));
    for j = 1:numel(LEYES)
        h(j) = plot(t_log, LOG.(LEYES{j}).TAU(:,i), '-', 'Color', colors(j,:), ...
                    'LineWidth', lw); hold on;
    end
    yline( TAU_MAX, '--k', 'LineWidth', 0.9, 'HandleVisibility', 'off');
    yline(-TAU_MAX, '--k', 'LineWidth', 0.9, 'HandleVisibility', 'off');
    if i == 1, ax1 = ax; h1 = h; end
    xlabel('Tiempo [s]', 'FontSize', fs);
    ylabel(sprintf('$\\tau_%d\\;[\\mathrm{N\\cdot m}]$', i), 'Interpreter', 'latex', 'FontSize', fs);
    title(jointNames{i}, 'FontSize', fs_title);
    ylim([-1.25*TAU_MAX, 1.25*TAU_MAX]);
    grid on; box on; set(gca, 'FontSize', fs); xlim(xlims);
end
lgd = legend(ax1, h1, nombres, 'Orientation', 'horizontal', 'FontSize', fs);
lgd.Layout.Tile = 'north';
title(tl, sprintf('[%s] Torques de control', mode_label), 'FontSize', 14, 'FontWeight', 'bold');

% ── Figura 4 — Superficies deslizantes ───────────────────────────────────────
figure(4); clf;
set(gcf, 'Color', 'w', 'Position', [160 160 1100 600]);
tl = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for i = 1:4
    ax = nexttile(tl);
    h = gobjects(1, numel(LEYES));
    for j = 1:numel(LEYES)
        h(j) = plot(t_log, LOG.(LEYES{j}).S(:,i), '-', 'Color', colors(j,:), ...
                    'LineWidth', lw); hold on;
    end
    yline(0, ':', 'LineWidth', 0.8, 'HandleVisibility', 'off');
    yline( PHI, '--k', 'LineWidth', 0.8, 'HandleVisibility', 'off');   % capa limite del MRAC
    yline(-PHI, '--k', 'LineWidth', 0.8, 'HandleVisibility', 'off');
    if i == 1, ax1 = ax; h1 = h; end
    xlabel('Tiempo [s]', 'FontSize', fs);
    ylabel(sprintf('$s_{%d}$ [rad/s]', i), 'Interpreter', 'latex', 'FontSize', fs);
    title(jointNames{i}, 'FontSize', fs_title);
    grid on; box on; set(gca, 'FontSize', fs); xlim(xlims);
end
lgd = legend(ax1, h1, nombres, 'Orientation', 'horizontal', 'FontSize', fs);
lgd.Layout.Tile = 'north';
title(tl, sprintf('[%s] Superficies deslizantes', mode_label), 'FontSize', 14, 'FontWeight', 'bold');

% ── Figura 5 — Parametros estimados del MRAC adaptativo ──────────────────────
AHm = LOG.MRAC.AH;
figure(5); clf;
set(gcf, 'Color', 'w', 'Position', [180 180 1200 720]);
tl = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
% (1) Escala de inercia alpha1..alpha3
nexttile(tl);
h = gobjects(1, 3);
for i = 1:3
    h(i) = plot(t_log, AHm(:,i), '-', 'Color', c_par(i,:), 'LineWidth', lw); hold on;
end
yline(mass_inertia_scale, '--k', 'LineWidth', 0.9, ...
      'Label', sprintf('verdadero (%.2f)', mass_inertia_scale), 'HandleVisibility', 'off');
xlabel('Tiempo [s]', 'FontSize', fs);
ylabel('$\hat{\alpha}_k$', 'Interpreter', 'latex', 'FontSize', fs);
title('Escala de inercia estimada (link2..link4)', 'FontSize', fs_title);
ylim_centro(gca, AHm(:,1:3), mass_inertia_scale, 0.05);
legend(h, {'$\hat{\alpha}_1$', '$\hat{\alpha}_2$', '$\hat{\alpha}_3$'}, ...
       'Interpreter', 'latex', 'Location', 'best', 'FontSize', fs);
grid on; box on; set(gca, 'FontSize', fs); xlim(xlims);
% (2) Carga: dm en el eje izquierdo, d(m·c) en el derecho
nexttile(tl);
yyaxis left;
h_dm = plot(t_log, AHm(:,4), '-', 'Color', c_par(1,:), 'LineWidth', lw); hold on;
yline(LOAD_TRUE(1), '--', 'Color', c_par(1,:), 'LineWidth', 0.9, 'HandleVisibility', 'off');
ylabel('$\Delta\hat{m}$ [kg]', 'Interpreter', 'latex', 'FontSize', fs);
set(gca, 'YColor', c_par(1,:));
ylim_centro(gca, AHm(:,4), LOAD_TRUE(1), 0.02);
yyaxis right;
h_mc = gobjects(1, 3);
for i = 1:3
    h_mc(i) = plot(t_log, AHm(:,4+i), '-', 'Color', c_par(i+1,:), 'LineWidth', lw); hold on;
    yline(LOAD_TRUE(1+i), '--', 'Color', c_par(i+1,:), 'LineWidth', 0.9, 'HandleVisibility', 'off');
end
ylabel('$\Delta\widehat{mc}$ [kg$\cdot$m]', 'Interpreter', 'latex', 'FontSize', fs);
set(gca, 'YColor', [0.3 0.3 0.3]);
ylim_centro(gca, [AHm(:,5:7); LOAD_TRUE(2:4)'], 0, 0.005);
xlabel('Tiempo [s]', 'FontSize', fs);
title('Carga estimada en el efector (-- verdadero)', 'FontSize', fs_title);
legend([h_dm, h_mc], {'$\Delta\hat{m}$', '$\Delta\widehat{mc}_x$', '$\Delta\widehat{mc}_y$', ...
       '$\Delta\widehat{mc}_z$'}, 'Interpreter', 'latex', 'Location', 'best', 'FontSize', fs);
grid on; box on; set(gca, 'FontSize', fs); xlim(xlims);
% (3) Friccion viscosa Fv1
nexttile(tl);
h_fv = plot(t_log, AHm(:,8), '-', 'Color', c_par(1,:), 'LineWidth', lw); hold on;
yline(FV1_TRUE, '--', 'Color', c_par(1,:), 'LineWidth', 0.9, 'HandleVisibility', 'off');
yline(Fv(1), ':', 'Color', [0.5 0.5 0.5], 'LineWidth', 0.9, 'Label', 'nominal', ...
      'HandleVisibility', 'off');
xlabel('Tiempo [s]', 'FontSize', fs);
ylabel('$\hat{F}_{v,1}$ [N$\cdot$m$\cdot$s/rad]', 'Interpreter', 'latex', 'FontSize', fs);
title('Friccion viscosa estimada, J1 (-- verdadero)', 'FontSize', fs_title);
legend(h_fv, {'$\hat{F}_{v,1}$'}, 'Interpreter', 'latex', 'Location', 'best', 'FontSize', fs);
grid on; box on; set(gca, 'FontSize', fs); xlim(xlims);
% (4) Friccion de Coulomb Fc1..Fc4
nexttile(tl);
h_fc = gobjects(1, 4);
for i = 1:4
    h_fc(i) = plot(t_log, AHm(:,8+i), '-', 'Color', c_par(i,:), 'LineWidth', lw); hold on;
    yline(FC_TRUE(i), '--', 'Color', c_par(i,:), 'LineWidth', 0.9, 'HandleVisibility', 'off');
end
yline(Fc(1), ':', 'Color', [0.5 0.5 0.5], 'LineWidth', 0.9, 'Label', 'nominal', ...
      'HandleVisibility', 'off');
xlabel('Tiempo [s]', 'FontSize', fs);
ylabel('$\hat{F}_{c,i}$ [N$\cdot$m]', 'Interpreter', 'latex', 'FontSize', fs);
title('Friccion de Coulomb estimada (-- verdadero)', 'FontSize', fs_title);
legend(h_fc, {'$\hat{F}_{c,1}$', '$\hat{F}_{c,2}$', '$\hat{F}_{c,3}$', '$\hat{F}_{c,4}$'}, ...
       'Interpreter', 'latex', 'Location', 'best', 'FontSize', fs);
grid on; box on; set(gca, 'FontSize', fs); xlim(xlims);
title(tl, sprintf('[%s] Parametros estimados del MRAC adaptativo', mode_label), ...
      'FontSize', 14, 'FontWeight', 'bold');

%% ── Exportacion ──────────────────────────────────────────────────────────────
if EXPORT_FIGS
    output_dir = fullfile(this_dir, 'plots', sprintf('act%d', ACTIVIDAD));
    if ~exist(output_dir, 'dir'), mkdir(output_dir); end
    fnames = {'q_tracking', 'q_error', 'torques', 'surfaces', 'params'};
    for f = 1:numel(fnames)
        if ishghandle(f)
            exportgraphics(figure(f), fullfile(output_dir, [fnames{f} '.png']), 'Resolution', 300);
            exportgraphics(figure(f), fullfile(output_dir, [fnames{f} '.eps']), ...
                           'ContentType', 'vector', 'Resolution', 600);
        end
    end
    % Vista 3D solo en PNG: en EPS vectorial, con las mallas STL, son ~43 MB
    if ~isempty(H) && isvalid(H.fig)
        exportgraphics(H.fig, fullfile(output_dir, 'robot3d_final.png'), 'Resolution', 300);
    end
    fprintf('Graficas guardadas en: %s\n', output_dir);
end

%% ═════════════════════════ Funciones locales ═════════════════════════════════

% ═══════════════════════════════════════════════════════════════════════════
%  SECCION 1 — Trayectoria articular de referencia  (COMPLETAR)
%  La misma de los nodos SMC y MRAC, de la forma:
%    qi_d(t)   = Ai + Bi*sin(w*t)
%    dqi_d(t)  = Bi*w*cos(w*t)
%    ddqi_d(t) = -Bi*w^2*sin(w*t)
%
%  Parametro de entrada:
%    t — tiempo actual de simulacion [s]
%
%  Salida esperada:
%    qd   = [q1_des(t);    q2_des(t);   q3_des(t);   q4_des(t)]     [rad]
%    dqd  = [dq1_des(t);   dq2_des(t);  dq3_des(t);  dq4_des(t)]    [rad/s]
%    ddqd = [ddq1_des(t);  ddq2_des(t); ddq3_des(t); ddq4_des(t)]   [rad/s^2]
% ═══════════════════════════════════════════════════════════════════════════
function [qd, dqd, ddqd] = trayectoria_deseada(t)
    w = 1.0;   % [rad/s] — no modificar
    % -------------------------------------------------------------------
    % COMPLETAR
    % -------------------------------------------------------------------
    qd   = zeros(4,1);
    dqd  = zeros(4,1);
    ddqd = zeros(4,1);
end

% ═══════════════════════════════════════════════════════════════════════════
%  SECCION 2 — Modelo nominal y regresor  (COMPLETAR)
%  Argumentos: q, dq (medidas), dqr, ddqr (referencias de Slotine-Li),
%  th = tanh(dq_d/eps) (4x1) y P (modelo nominal: P.FV0, P.FC0, P.PI_NOM).
% ═══════════════════════════════════════════════════════════════════════════

% 2.1 Torque nominal de Slotine-Li con la friccion nominal:
%     tau_nom = M(q)*ddq_r + C(q,dq)*dq_r + g(q) + Fv0.*dq + Fc0.*tanh(dq_d/eps)
function tau_nom = tau_nominal(q, dq, dqr, ddqr, th, P)
    [M, C, g] = modelo_nominal(q, dq);   % M(q), C(q,dq), g(q) del URDF (no modificar)
    % Friccion nominal: P.FV0 = Fv0 (4x1) y P.FC0 = Fc0 (4x1)

    tau_nom = zeros(4,1);   % COMPLETAR
end

% Regresor 4x12: columnas en el orden de a_hat
%   [alpha1..3 | dm, dmcx, dmcy, dmcz | Fv1 | Fc1..4]
function Y = regresor(q, dq, dqr, ddqr, th, P)
    % Regresor por cuerpo: para el cuerpo movido por joint k (k = 1..4), su
    % aporte al torque es Yb(:,:,k)*pi_k, con
    % pi_k (10x1) = [m, m*cx, m*cy, m*cz, Ixx, Ixy, Iyy, Ixz, Iyz, Izz].
    % P.PI_NOM(:,k) tiene los valores nominales de pi_k (URDF).
    Yb = regresor_cuerpos(q, dq, dqr, ddqr);
    Y  = zeros(4, 12);

    % Columnas de la carga theta_load = [dm, dmcx, dmcy, dmcz]: la carga suma
    % masa y primer momento al ultimo cuerpo (k = 4)
    Y(:, 4:7) = Yb(:, 1:4, 4);

    % 2.2 Columnas de alpha1..alpha3 (cuerpos k = 1..3): alpha_k escala todos
    %     los parametros del cuerpo, pi_k = alpha_k*P.PI_NOM(:,k)
    %     Y(:, k) = ...
    % COMPLETAR

    % 2.3 Columna de Fv1 (friccion viscosa de joint1: Fv1*dq1)
    %     Y(1, 8) = ...
    % COMPLETAR

    % 2.4 Columnas de Fc1..Fc4 (Coulomb de cada articulacion: Fc_j*tanh(dq_d,j/eps))
    %     Y(j, 8 + j) = ...
    % COMPLETAR
end
% ═══════════════════════════════════════════════════════════════════════════

% ── Autoverificacion de la SECCION 2 ──────────────────────────────────────────
% Compara tau_nominal() con la dinamica rigida de los cuerpos y cada bloque de
% regresor() con el cambio del torque al perturbar esos parametros, en tres
% estados de prueba (dq ~= dq_r para distinguir C(q,dq)*dq_r de C(q,dq)*dq).
function autoverificacion(P)
    est = { ...
        [0.3; 0.5; -0.4; 0.8],   [0.6; -0.4; 0.7; -0.5], [-0.2; 0.9; -0.3; 0.6], ...
        [1.1; -0.7; 0.5; -1.3],  [0.6; -0.3; 0.9; -0.7];
        [-0.7; -0.2; 0.6; -0.3], [-0.3; 0.8; -0.6; 0.4], [0.5; -0.4; 0.2; -0.9], ...
        [-0.6; 1.2; -0.8; 0.4],  [-0.4; 0.8; -0.2; 0.5];
        [0.0; 0.931; 0.35; -1.49], [0.2; 0.1; -0.3; 0.6], [0.1; 0.3; -0.5; 0.2], ...
        [0.3; 0.2; -0.1; 0.7],   [0.95; -0.95; 0.1; 0.0]};
    d_alpha = [0.15; -0.10; 0.20];               % cambios de prueba de los parametros
    d_load  = [0.05; 0.004; -0.002; 0.003];
    d_fv    = 0.01;
    d_fc    = [0.02; -0.01; 0.03; 0.015];
    TOL     = 1e-9;   % [N·m] las dos formas deben coincidir a precision de maquina

    e = zeros(1, 5); z = true(1, 5); e_cdq = 0; e_nofric = 0;
    PI0 = P.PI_NOM;
    for n = 1:size(est, 1)
        [q, dq, dqr, ddqr, th] = est{n, :};
        tr0  = tau_rigido(q, dq, dqr, ddqr, PI0);
        fric = P.FV0.*dq + P.FC0.*th;

        % Torque nominal
        tn = tau_nominal(q, dq, dqr, ddqr, th, P);
        [M, phib] = OMDyn(q, dq);
        e(1)     = max(e(1), max(abs(tn - (tr0 + fric))));
        e_cdq    = max(e_cdq, max(abs(tn - (M*ddqr + phib + fric))));
        e_nofric = max(e_nofric, max(abs(tn - tr0)));
        z(1)     = z(1) && all(tn == 0);

        % Regresor, bloque por bloque
        Y = regresor(q, dq, dqr, ddqr, th, P);
        PIa = PI0;
        PIa(:, 1:3) = PI0(:, 1:3) .* (1 + d_alpha');           % alpha_k escala el cuerpo k
        PIl = PI0;
        PIl(1:4, 4) = PI0(1:4, 4) + d_load;                    % carga en el ultimo cuerpo
        d_ref = {tau_rigido(q, dq, dqr, ddqr, PIa) - tr0, ...
                 [d_fv*dq(1); 0; 0; 0], ...
                 d_fc.*th, ...
                 tau_rigido(q, dq, dqr, ddqr, PIl) - tr0};
        cols  = {1:3, 8, 9:12, 4:7};
        delta = {d_alpha, d_fv, d_fc, d_load};
        for b = 1:4
            e(b+1) = max(e(b+1), max(abs(Y(:, cols{b})*delta{b} - d_ref{b})));
            z(b+1) = z(b+1) && all(Y(:, cols{b}) == 0, 'all');
        end
    end

    pista = '';
    if e_cdq <= TOL
        pista = ': usa C(q,dq)*dq; Slotine-Li usa C(q,dq)*dq_r';
    elseif e_nofric <= TOL
        pista = ': falta la friccion nominal';
    end
    nombres = {'tau_nom (2.1)', 'regresor: columnas alpha1..alpha3 (2.2)', ...
               'regresor: columna Fv1 (2.3)', 'regresor: columnas Fc1..Fc4 (2.4)', ...
               'regresor: columnas de la carga (no modificar)'};
    ok = true;
    fprintf('Autoverificacion de la SECCION 2 (modelo nominal y regresor):\n');
    for b = 1:5
        if e(b) <= TOL
            fprintf('  %-46s OK\n', nombres{b});
        elseif z(b)
            ok = false;
            fprintf('  %-46s SIN COMPLETAR\n', nombres{b});
        else
            ok = false;
            if b == 1, p = pista; else, p = ''; end
            fprintf('  %-46s ERROR (max |dif| = %.3g)%s\n', nombres{b}, e(b), p);
        end
    end
    if ~ok
        error('SECCION 2 incompleta o incorrecta: revisar el informe de la autoverificacion.');
    end
    fprintf('\n');
end

% Torque de Slotine-Li del modelo rigido con los parametros PI (10x4) de los
% cuerpos movidos por joint1..joint4
function tau = tau_rigido(q, dq, dqr, ddqr, PI)
    Yb  = regresor_cuerpos(q, dq, dqr, ddqr);
    tau = zeros(4, 1);
    for k = 1:4
        tau = tau + Yb(:, :, k) * PI(:, k);
    end
end

% ── Modelo nominal: M(q), C(q,dq) y g(q) desde OMDyn ──────────────────────────
% OMDyn entrega M(q) y phib(q,dq) = C(q,dq)*dq + g(q). C se obtiene por
% polarizacion, columna a columna: C(:,j) = (phib(dq+e_j) - phib(dq) - phib(e_j) + g)/2
% (la C de los simbolos de Christoffel, la misma del nodo).
function [M, C, g] = modelo_nominal(q, dq)
    [M, phib] = OMDyn(q, dq);
    [~, g]    = OMDyn(q, zeros(4,1));
    C = zeros(4);
    for j = 1:4
        ej = zeros(4,1); ej(j) = 1;
        [~, p_sum] = OMDyn(q, dq + ej);
        [~, p_ej]  = OMDyn(q, ej);
        C(:, j) = 0.5*(p_sum - phib - p_ej + g);
    end
end

% ── Regresor de Slotine-Li por cuerpo ───────────
% Yb(:,:,k) (4x10): tau = sum_k Yb(:,:,k)*pi_k para
% tau = M(q)*ddq_r + C(q,dq)*dq_r + g(q). Polarizacion del regresor de RNEA:
%   Y_SL = R(q,0,ddq_r) + (R(q,dq+dq_r,0) - R(q,dq,0) - R(q,dq_r,0) + R(q,0,0))/2
function Yb = regresor_cuerpos(q, dq, dqr, ddqr)
    z  = zeros(4,1);
    Yb = regresor_rnea(q, z, ddqr) + 0.5*( regresor_rnea(q, dq + dqr, z) ...
         - regresor_rnea(q, dq, z) - regresor_rnea(q, dqr, z) + regresor_rnea(q, z, z) );
end

% Regresor de tau = RNEA(q, v, a) con gravedad, por cuerpo (algebra espacial)
function Yb = regresor_rnea(q, v, a)
    PJ = [0.012 0 0.017; 0 0 0.0595; 0.024 0 0.128; 0.124 0 0]';   % origenes de joint1..4
    AX = [0 0 1; 0 1 0; 0 1 0; 0 1 0]';                             % ejes de joint1..4
    X  = zeros(6, 6, 4); Sj = zeros(6, 4); A = zeros(6, 10, 4);
    vp = zeros(6,1);
    ap = [0; 0; 0; 0; 0; 9.81];   % gravedad como aceleracion de la base
    for k = 1:4
        c = cos(q(k)); s = sin(q(k));
        if AX(3,k) == 1
            R = [c -s 0; s c 0; 0 0 1];
        else
            R = [c 0 s; 0 1 0; -s 0 c];
        end
        E = R';
        X(:,:,k) = [E zeros(3); -E*skew(PJ(:,k)) E];
        Sj(:,k)  = [AX(:,k); 0; 0; 0];
        vk = X(:,:,k)*vp + Sj(:,k)*v(k);
        ak = X(:,:,k)*ap + Sj(:,k)*a(k) + crm(vk)*Sj(:,k)*v(k);
        A(:,:,k) = regresor_cuerpo(vk, ak);
        vp = vk; ap = ak;
    end
    Yb = zeros(4, 10, 4);
    for k = 1:4
        W = A(:,:,k);
        for i = k:-1:1
            Yb(i,:,k) = Sj(:,i)'*W;
            W = X(:,:,i)'*W;
        end
    end
end

% Fuerza espacial del cuerpo f = I*a + v x* (I*v) = A(v,a)*pi (6x10)
function A = regresor_cuerpo(v, a)
    w = v(1:3); u = v(4:6); al = a(1:3); b = a(4:6);
    Sw = skew(w);
    A = [zeros(3,1),       -skew(b) - Sw*skew(u) + skew(u)*Sw, Lmat(al) + Sw*Lmat(w);
         b + cross(w, u),   skew(al) + Sw*Sw,                  zeros(3,6)];
end

function L = Lmat(w)   % I*w = Lmat(w)*[Ixx Ixy Iyy Ixz Iyz Izz]'
    L = [w(1) w(2) 0    w(3) 0    0;
         0    w(1) w(2) 0    w(3) 0;
         0    0    0    w(1) w(2) w(3)];
end

function S = skew(x)
    S = [0 -x(3) x(2); x(3) 0 -x(1); -x(2) x(1) 0];
end

function X = crm(v)    % producto cruz de vectores de movimiento
    X = [skew(v(1:3)) zeros(3); skew(v(4:6)) skew(v(1:3))];
end

% ── Parametros dinamicos de los cuerpos (URDF, como en OMDyn) ─────────────────
% Columna k: cuerpo movido por joint k, pi_k = [m, m*c, Ixx, Ixy, Iyy, Ixz,
% Iyz, Izz] con la inercia en el origen del frame del cuerpo. El cuerpo 4
% (link5) incluye los placeholders del gripper, como OMDyn.
function PI = pi_cuerpos()
    m  = [0.0970; 0.1235; 0.1151; 0.2206];
    rc = [0.000000 0.000657 0.044319;
          0.007606 0.000389 0.099924;
          0.089168 0.000437 0.000001;
          0.062451 0.000000 0.005878];
    Ic = {[ 3.078317e-05  0             0;
            0             2.670366e-05 -1.289659e-07;
            0            -1.289659e-07  1.627175e-05], ...
          [ 1.979680e-04 -2.511443e-07 -2.497601e-05;
           -2.511443e-07  2.046627e-04 -1.351040e-06;
           -2.497601e-05 -1.351040e-06  3.374865e-05], ...
          [ 1.930305e-05 -1.189078e-06  3.461654e-09;
           -1.189078e-06  1.483284e-04  0;
            3.461654e-09  0             1.547550e-04], ...
          [ 1.344328e-04 -1.788649e-08  2.259163e-05;
           -1.788649e-08  1.892619e-04  9.936937e-07;
            2.259163e-05  9.936937e-07  2.174510e-04]};
    m_g = [1e-3; 1e-3; 1e-3];                          % placeholders del gripper
    p_g = [0.0817 0.021 0; 0.0817 -0.021 0; 0.126 0 0];
    PI = zeros(10, 4);
    for k = 1:4
        c  = rc(k,:)';
        mk = m(k);
        h  = m(k)*c;
        Io = Ic{k} + m(k)*((c'*c)*eye(3) - c*c');      % inercia en el origen
        if k == 4
            for gg = 1:3
                pg = p_g(gg,:)';
                mk = mk + m_g(gg);
                h  = h + m_g(gg)*pg;
                Io = Io + 1e-6*eye(3) + m_g(gg)*((pg'*pg)*eye(3) - pg*pg');
            end
        end
        PI(:,k) = [mk; h; Io(1,1); Io(1,2); Io(2,2); Io(1,3); Io(2,3); Io(3,3)];
    end
end

% ── Eje y centrado en ref con semiancho minimo semi (crece si los datos salen) ─
% Evita que variaciones de 0.1 % parezcan grandes por el zoom automatico.
function ylim_centro(ax, datos, ref, semi)
    lo  = min([datos(:); ref - semi]);
    hi  = max([datos(:); ref + semi]);
    pad = 0.05*(hi - lo);
    ylim(ax, [lo - pad, hi + pad]);
end

% ── Funcion de conmutacion sat(s/phi), elemento a elemento ────────────────────
function r = rho_vec(s, phi)
    r = max(-1, min(1, s / phi));
end

% ── Dinamica de la planta: ms*M*ddq + ms*phib + friccion articular = tau ──────
function dx = dinamica_planta(x, tau, b, fc, eps_pl, ms)
    q  = x(1:4);
    dq = x(5:8);
    [M, phib] = OMDyn(q, dq);
    ddq = (ms*M) \ (tau - ms*phib - b.*dq - fc.*tanh(dq / eps_pl));
    dx = [dq; ddq];
end

% ── Paso RK4 con torque constante (ZOH) ───────────────────────────────────────
function x = rk4_step(x, tau, h, b, fc, eps_pl, ms)
    k1 = dinamica_planta(x,            tau, b, fc, eps_pl, ms);
    k2 = dinamica_planta(x + 0.5*h*k1, tau, b, fc, eps_pl, ms);
    k3 = dinamica_planta(x + 0.5*h*k2, tau, b, fc, eps_pl, ms);
    k4 = dinamica_planta(x + h*k3,     tau, b, fc, eps_pl, ms);
    x  = x + (h/6)*(k1 + 2*k2 + 2*k3 + k4);
end

% ── Cinematica directa analitica del efector ──────────────────────────────────
function y = fkin_local(q)
    x_base = 0.012;  z_base = 0.017 + 0.0595;
    x23 = 0.024;  z23 = 0.128;
    l34 = 0.124;  l4e = 0.126;
    r = x23*cos(q(2)) + z23*sin(q(2)) ...
      + l34*cos(q(2)+q(3)) + l4e*cos(q(2)+q(3)+q(4));
    z = z_base + (-x23*sin(q(2)) + z23*cos(q(2))) ...
      - l34*sin(q(2)+q(3)) - l4e*sin(q(2)+q(3)+q(4));
    y = [x_base + r*cos(q(1)); r*sin(q(1)); z; q(2)+q(3)+q(4)];
end

% ── Vista 3D del robot: STLs del URDF sobre una jerarquia de hgtransform ──────
% Cadena (urdf/open_manipulator_x.urdf):
%   link1 (fijo) --joint1 z @ [0.012 0 0.017]--> link2 (malla +0.019 z)
%         --joint2 y @ [0 0 0.0595]--> link3
%         --joint3 y @ [0.024 0 0.128]--> link4
%         --joint4 y @ [0.124 0 0]--> link5 + palmas del gripper @ [0.0817 ±0.021 0]
function H = build_robot3d(mesh_dir, q0, y_des_path)
    H.fig = figure(7); clf(H.fig);
    set(H.fig, 'Color', 'w', 'Name', 'OpenMANIPULATOR-X — Prelab MRAC', ...
        'Position', [80 80 900 700]);
    H.ax = axes(H.fig); hold(H.ax, 'on');
    grid(H.ax, 'on'); box(H.ax, 'on'); axis(H.ax, 'equal');
    xlim(H.ax, [-0.15 0.45]); ylim(H.ax, [-0.30 0.30]); zlim(H.ax, [-0.01 0.3]);
    xlabel(H.ax, 'x [m]'); ylabel(H.ax, 'y [m]'); zlabel(H.ax, 'z [m]');
    view(H.ax, 45, 25);
    title(H.ax, 'MRAC adaptativo articular — OpenMANIPULATOR-X');

    % piso de referencia
    patch(H.ax, 'XData', [-0.15 0.45 0.45 -0.15], ...
                'YData', [-0.30 -0.30 0.30 0.30], 'ZData', zeros(1,4), ...
          'FaceColor', [0.94 0.94 0.94], 'EdgeColor', [0.85 0.85 0.85], ...
          'HandleVisibility', 'off');

    % jerarquia: ax -> T1 -> T2 -> T3 -> T4 (cada Matrix es LOCAL a su padre)
    H.T1 = hgtransform('Parent', H.ax);
    H.T2 = hgtransform('Parent', H.T1);
    H.T3 = hgtransform('Parent', H.T2);
    H.T4 = hgtransform('Parent', H.T3);

    c_link = [0.45 0.45 0.48];
    c_grip = [0.25 0.25 0.28];
    add_stl(H.ax, mesh_dir, 'link1.stl', [0 0 0],     c_link);
    add_stl(H.T1, mesh_dir, 'link2.stl', [0 0 0.019], c_link);
    add_stl(H.T2, mesh_dir, 'link3.stl', [0 0 0],     c_link);
    add_stl(H.T3, mesh_dir, 'link4.stl', [0 0 0],     c_link);
    add_stl(H.T4, mesh_dir, 'link5.stl', [0 0 0],     c_link);
    add_stl(H.T4, mesh_dir, 'gripper_left_palm.stl',  [0.0817  0.021 0], c_grip);
    add_stl(H.T4, mesh_dir, 'gripper_right_palm.stl', [0.0817 -0.021 0], c_grip);

    % punto EE (l4e = 0.126 en el marco de link5), trayectorias y reloj
    line('Parent', H.T4, 'XData', 0.126, 'YData', 0, 'ZData', 0, ...
         'Marker', 'o', 'MarkerSize', 5, 'Color', [0 0.447 0.741], ...
         'MarkerFaceColor', [0 0.447 0.741], 'HandleVisibility', 'off');
    plot3(H.ax, y_des_path(:,1), y_des_path(:,2), y_des_path(:,3), '--', ...
          'Color', [0.85 0.325 0.098], 'LineWidth', 1.2, ...
          'DisplayName', 'Trayectoria deseada');
    H.trace = animatedline(H.ax, 'Color', [0 0.447 0.741], 'LineWidth', 1.6, ...
                           'DisplayName', 'Trayectoria simulada');
    legend(H.ax, 'Location', 'southoutside', 'Orientation', 'horizontal');
    H.txt = text(H.ax, -0.14, -0.29, 0.28, 't = 0.00 s', 'FontSize', 12, ...
                 'FontWeight', 'bold');

    camlight(H.ax, 'headlight');
    lighting(H.ax, 'gouraud');
    update_robot3d(H, q0, 0);
end

function add_stl(parent, mesh_dir, fname, offset, col)
    TR = stlread(fullfile(mesh_dir, fname));
    V  = TR.Points * 1e-3 + offset(:)';   % STL en mm (scale 0.001 del URDF)
    patch('Parent', parent, 'Faces', TR.ConnectivityList, 'Vertices', V, ...
          'FaceColor', col, 'EdgeColor', 'none', 'FaceLighting', 'gouraud', ...
          'AmbientStrength', 0.45, 'HandleVisibility', 'off');
end

function update_robot3d(H, q, t)
    H.T1.Matrix = makehgtform('translate', [0.012 0 0.017]) * makehgtform('zrotate', q(1));
    H.T2.Matrix = makehgtform('translate', [0 0 0.0595])    * makehgtform('yrotate', q(2));
    H.T3.Matrix = makehgtform('translate', [0.024 0 0.128]) * makehgtform('yrotate', q(3));
    H.T4.Matrix = makehgtform('translate', [0.124 0 0])     * makehgtform('yrotate', q(4));
    set(H.txt, 'String', sprintf('t = %.2f s', t));
end

function trace_add(H, q)
% Agrega la pose q a la traza del efector final (sin mover el robot).
    y = fkin_local(q);
    addpoints(H.trace, y(1), y(2), y(3));
end
