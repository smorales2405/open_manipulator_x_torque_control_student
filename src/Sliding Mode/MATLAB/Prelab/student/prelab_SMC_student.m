%% prelab_SMC_student.m
% Prelab — Control por Modo Deslizante (SMC) articular
% OpenMANIPULATOR-X — simulacion MATLAB
%
% Plantilla de estudiante:
% Se deben completar:
%
%   SECCION 1 — Trayectoria de referencia articular   (funcion trayectoria_deseada)
%   SECCION 2 — Ganancias del controlador              (LAMBDA_Q, K_V, K_S, phi, TAU_MAX)
%   SECCION 3 — Ley de control                         (dentro del lazo a 200 Hz y
%                                                       funcion rho_vec: sat y sign)
%
% Infraestructura proporcionada:
%   - Carga de OMDyn (MEX) y calculo de M(q), phib(q,dq)
%   - Feedforward de friccion articular (SIEMPRE activo)
%   - Planta simulada: integracion RK4 con ZOH del torque
%   - Saturacion de torque, verificacion de limites articulares
%   - Vista 3D del robot (URDF + STLs), figuras de seguimiento y metricas
%   - Parametros del lazo (T_SIM, Ts, N_SUB, T_REG, Fv, Fc, limites)
%
% Planta simulada:
%   ms*M(q)*ddq + ms*phib(q,dq) + ds*Fv.*dq + Fc.*tanh(dq/eps) = tau_sat
%     ms = mass_inertia_scale   ds = damping_scale
%   integrada con RK4 a Ts/N_SUB, con ZOH del torque durante Ts = 1/200 s.
%
% Visualizacion 3D del robot (cadena del URDF + STLs de meshes/), tres opciones:
%   ANIM_MODE = 'post' — simula primero todo el horizonte y luego anima la
%               trayectoria con dt = 1/200 s entre movimientos (tiempo real).
%   ANIM_MODE = 'live' — renderiza DURANTE la simulacion.
%   ANIM_MODE = 'off'  — sin animacion (solo figuras de seguimiento).
%
% Figuras de seguimiento: (posiciones, errores, superficies y
% torques) + tabla de metricas en consola (sin el transitorio inicial).
%

clear; clc; close all;

%% ── Configuracion ────────────────────────────────────────────────────────────
T_SIM       = 25.0;    % [s] duracion de la simulacion
ANIM_MODE   = 'off';   % 'post' | 'live' | 'off'

EXPORT_FIGS = false;   % true = guardar PNG (300 dpi) y EPS vectorial (600 dpi) en
                       %        plots/ junto al script

rho_func    = 'sat';   % funcion de conmutacion: 'sat' | 'sign'

% ── Escalas de la planta ─────────
% Solo afectan a la planta simulada; el controlador usa el modelo nominal.
mass_inertia_scale = 1.0;   % factor sobre masas e inercias de los eslabones
damping_scale      = 1.0;   % factor sobre la friccion viscosa Fv

Ts          = 1/200;   % [s] periodo del lazo de control (200 Hz)
T_REG       = 1.0;     % [s] ventana excluida de las metricas: transitorio
                       %     inicial (el robot parte en reposo y dq_des(0)~=0)

% ── Friccion articular de la planta ─────────
% La planta la simula y el controlador la compensa via feedforward.
Fv = 0.001 * ones(4,1);   % Damping Friction [N·m·s/rad]
Fc = 0.05  * ones(4,1);   % Coulomb Friction [N·m]
F_eps     = 0.05;   % [rad/s] suavizado del tanh en el feedforward de Coulomb
eps_plant = 0.01;   % [rad/s] suavizado del sign() de Coulomb en la planta
N_SUB     = 100;    % subpasos RK4 por tick (h = Ts/N_SUB = 0.05 ms). Coulomb es rigido
                    % cerca de dq = 0 y RK4 necesita N_SUB >= 38 para ser estable

% Limites articulares del URDF
Q_LOWER = [-2.3562; -1.9199; -1.9199; -1.8];
Q_UPPER = [ 2.3562;  1.7453;  1.5708;  2.1];

%% ── Rutas y dependencias (autocontenidas en esta carpeta) ────────────────────
this_dir = fileparts(mfilename('fullpath'));
mesh_dir = fullfile(this_dir, 'meshes');
addpath(this_dir);

assert(~isempty(which('OMDyn')), ...
    'OMDyn.%s no encontrado junto al script para esta plataforma.', mexext);
assert(isfolder(mesh_dir), 'No se encontro la carpeta meshes/ junto al script.');
assert(any(strcmp(rho_func, {'sat', 'sign'})), ...
    'rho_func debe ser ''sat'' o ''sign''.');

fprintf('Modelo OMDyn: %s\n', which('OMDyn'));
fprintf(['Feedforward de friccion: damping=[%.4f %.4f %.4f %.4f]  ' ...
    'coulomb=[%.4f %.4f %.4f %.4f]\n'], Fv, Fc);
fprintf('Escalas de la planta: masa=%.2f  damping=%.2f\n', ...
    mass_inertia_scale, damping_scale);
fprintf('Tiempo de simulacion: %.1f s  |  lazo: %.0f Hz  |  animacion: %s\n\n', ...
    T_SIM, 1/Ts, ANIM_MODE);


%% ── Seccion 1: Referencia precomputada ─────────────────

Nsim  = round(T_SIM / Ts);
t_log = (0:Nsim)' * Ts;

QD = zeros(Nsim+1, 4); DQD = zeros(Nsim+1, 4); DDQD = zeros(Nsim+1, 4);
for k = 1:Nsim+1
    [qd, dqd, ddqd] = trayectoria_deseada(t_log(k));
    QD(k,:) = qd'; DQD(k,:) = dqd'; DDQD(k,:) = ddqd';
end

% Condicion inicial de la planta: EN la posicion inicial de la trayectoria y
% en reposo.
q0  = QD(1,:)';
dq0 = zeros(4,1);
fprintf('Pose inicial = q_des(0) = [%.3f %.3f %.3f %.3f] rad (en reposo)\n\n', q0);

% Trayectoria cartesiana deseada del EE (para la vista 3D)
Y_DES = zeros(Nsim+1, 4);
for k = 1:Nsim+1
    Y_DES(k,:) = fkin_local(QD(k,:)')';
end

%% ── Seccion 2: Vista 3D (solo modo 'live': se construye antes del lazo) ──────
H = [];
if strcmp(ANIM_MODE, 'live')
    H = build_robot3d(mesh_dir, q0, Y_DES);
    ANIM_DECIM = 4;            % render cada 4 ticks (~50 fps)
end

% ═══════════════════════════════════════════════════════════════════════════
%  SECCION 2 — GANANCIAS DEL CONTROLADOR  (COMPLETAR)
% -------------------------------------------------------------------
%  Indice de cada elemento:  [joint1, joint2, joint3, joint4]
LAMBDA_Q = [0.0; 0.0; 0.0; 0.0];   % COMPLETAR
K_V      = [0.0; 0.0; 0.0; 0.0];   % COMPLETAR
K_S      = [0.0; 0.0; 0.0; 0.0];   % COMPLETAR
phi      = 0.0;                    % COMPLETAR  [rad/s] capa limite de sat (no se usa con 'sign')
TAU_MAX  = 0.0;                    % COMPLETAR  [N·m]
% ═══════════════════════════════════════════════════════════════════════════

fprintf('Lambda=[%.1f %.1f %.1f %.1f]  Kv=[%.1f %.1f %.1f %.1f]  Ks=[%.1f %.1f %.1f %.1f]\n', ...
    LAMBDA_Q, K_V, K_S);
fprintf('rho=%s  phi=%.3f  tau_max=%.2f N·m\n\n', rho_func, phi, TAU_MAX);

%% ── Seccion 3: Lazo de control a 200 Hz ──────────────────────
Q = zeros(Nsim+1, 4); DQ = zeros(Nsim+1, 4); TAU = zeros(Nsim+1, 4); S = zeros(Nsim+1, 4);

Fv_plant = damping_scale * Fv;   % friccion viscosa de la planta

x = [q0; dq0];
t0_wall = tic;
for k = 1:Nsim
    % ── Leer estado ───────────────────────────
    q  = x(1:4);
    dq = x(5:8);

    % ── Referencia del tick ───────────────────────────────────────────────
    qd = QD(k,:)'; dqd = DQD(k,:)'; ddqd = DDQD(k,:)';

    % ── Matriz de masa M(q) y efectos no lineales phib(q,dq) ─
    [M, phib] = OMDyn(q, dq);

    % ══════════════════════════════════════════════════════════════════════
    %  SECCION 3 — LEY DE CONTROL  (COMPLETAR)
    %  Funcion de conmutacion: rho_vec(s_q, rho_func, phi)
    % -------------------------------------------------------------------
    e_q = zeros(4,1);
    s_q = zeros(4,1);
    tau = zeros(4,1);

    % ══════════════════════════════════════════════════════════════════════

    % ── Feedforward de friccion
    tau = tau + Fv.*dq + Fc.*tanh(dqd / F_eps);

    % ── Saturacion de torque ──────────────────────────────────────────────
    tau_sat = min(max(tau, -TAU_MAX), TAU_MAX);

    % ── Registro ──────────────────────────────
    Q(k,:) = q'; DQ(k,:) = dq'; S(k,:) = s_q'; TAU(k,:) = tau_sat';

    % ── Planta: RK4 con ZOH del torque ──────────────
    h = Ts / N_SUB;
    for i_sub = 1:N_SUB
        x = rk4_step(x, tau_sat, h, Fv_plant, Fc, eps_plant, mass_inertia_scale);
    end

    % ── Animacion en vivo + ritmo de tiempo real (solo 'live') ───────────
    if strcmp(ANIM_MODE, 'live')
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
        fprintf('t=%5.2f s  |s|=%.5f  |e|=%.5f rad  tau=[%+.3f %+.3f %+.3f %+.3f] Nm\n', ...
                t_log(k), norm(s_q), norm(e_q), tau_sat);
    end
end

Q(Nsim+1,:)   = x(1:4)';
DQ(Nsim+1,:)  = x(5:8)';
TAU(Nsim+1,:) = TAU(Nsim,:);
S(Nsim+1,:)   = S(Nsim,:);

fprintf('Simulacion completada (%.1f s).\n\n', T_SIM);

% Verificacion de limites articulares del URDF
for i = 1:4
    if any(Q(:,i) < Q_LOWER(i)) || any(Q(:,i) > Q_UPPER(i))
        warning('q%d salio de los limites del URDF [%.3f, %.3f] rad.', ...
                i, Q_LOWER(i), Q_UPPER(i));
    end
end

%% ── Seccion 4: Animacion 3D posterior (dt = 1/200 s por frame) ─────
if strcmp(ANIM_MODE, 'post')
    H = build_robot3d(mesh_dir, q0, Y_DES);
    drawnow;
    t0_wall = tic;
    k = 1;
    while k <= Nsim
        update_robot3d(H, Q(k,:)', t_log(k));
        drawnow;
        k_next = max(k + 1, 1 + floor(toc(t0_wall) / Ts));
        for k_skip = k:min(k_next - 1, Nsim+1)
            trace_add(H, Q(k_skip,:)');
        end
        k = k_next;
    end
    update_robot3d(H, Q(end,:)', t_log(end));
    trace_add(H, Q(end,:)');
    drawnow;
end

%% ── Seccion 5: Figuras de seguimiento ────────
e_q_log = Q - QD;

% Metricas por articulacion, excluyendo el transitorio inicial (T_REG)
m_reg = t_log >= T_REG;
if ~any(m_reg)
    warning('No hay muestras con t >= %.1f s; revisar T_REG.', T_REG);
end
fprintf('%s\n', repmat('═', 1, 88));
fprintf(' Metricas SMC articular  [Prelab | rho=%s | escalas %.2f/%.2f]  (t >= %.1f s)\n', ...
        rho_func, mass_inertia_scale, damping_scale, T_REG);
fprintf('%s\n', repmat('═', 1, 88));
fprintf('%-6s  %-12s  %-12s  %-14s  %-14s  %-10s  %-10s\n', ...
        'Joint', 'e_max[rad]', 'e_RMS[rad]', 'max|tau|[N·m]', 'tau_RMS[N·m]', 'Sat[%]', 'TV(tau)');
fprintf('%s\n', repmat('-', 1, 88));
for i = 1:4
    e_i   = e_q_log(m_reg, i);
    tau_i = TAU(m_reg, i);
    fprintf('q%-5d  %12.5f  %12.5f  %14.4f  %14.4f  %10.2f  %10.4f\n', i, ...
            max(abs(e_i)), sqrt(mean(e_i.^2)), ...
            max(abs(tau_i)), sqrt(mean(tau_i.^2)), ...
            100 * mean(abs(tau_i) >= 0.99*TAU_MAX), ...
            sum(abs(diff(tau_i))));
end
fprintf('%s\n\n', repmat('═', 1, 88));

lw         = 1.6;
fs         = 11;
fs_title   = 12;
color_ref  = [0.8500 0.3250 0.0980];   % naranja — referencia
color_meas = [0.0000 0.4470 0.7410];   % azul    — simulacion
color_surf = [0.4940 0.1840 0.5560];   % violeta — superficies deslizantes
color_tau  = [0.4660 0.6740 0.1880];   % verde   — torque
jointNames = {'Articulacion 1', 'Articulacion 2', ...
              'Articulacion 3', 'Articulacion 4'};
xlims      = [t_log(1), t_log(end)];
if strcmp(rho_func, 'sat')
    rho_label = sprintf('sat(s/%.2f)', phi);
else
    rho_label = 'sign(s)';
end
mode_label = sprintf('Prelab | rho = %s | escalas %.1f/%.1f', rho_label, ...
                     mass_inertia_scale, damping_scale);

% ── Figura 1 — Posiciones articulares ────────────────────────────────────────
figure(1); clf;
set(gcf, 'Color', 'w', 'Position', [100 100 1100 700]);
tl1  = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
axs1 = gobjects(1, 4);
h_ref1 = []; h_mea1 = [];
for i = 1:4
    axs1(i) = nexttile(tl1);
    h2 = plot(t_log, Q(:,i),  '-',  'Color', color_meas, 'LineWidth', lw); hold on;
    h1 = plot(t_log, QD(:,i), '--', 'Color', color_ref,  'LineWidth', lw);
    if i == 1; h_ref1 = h1; h_mea1 = h2; end
    xlabel('Tiempo [s]', 'FontSize', fs);
    ylabel(sprintf('$q_%d$ [rad]', i), 'Interpreter', 'latex', 'FontSize', fs);
    title(jointNames{i}, 'FontSize', fs_title);
    grid on; box on; set(gca, 'FontSize', fs); xlim(xlims);
end
lgd1 = legend(axs1(1), [h_ref1, h_mea1], {'Referencia', 'Simulacion'}, ...
              'Orientation', 'horizontal', 'FontSize', fs, 'Location', 'northoutside');
lgd1.Layout.Tile = 'north';
title(tl1, sprintf('[%s] SMC - Seguimiento de posiciones articulares', mode_label), ...
      'FontSize', 14, 'FontWeight', 'bold');

% ── Figura 2 — Errores de seguimiento articular ──────────────────────────────
figure(2); clf;
set(gcf, 'Color', 'w', 'Position', [120 120 1100 560]);
tl2 = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for i = 1:4
    nexttile(tl2);
    plot(t_log, e_q_log(:,i), '-', 'Color', color_meas, 'LineWidth', lw);
    yline(0, ':', 'LineWidth', 0.8);
    xlabel('Tiempo [s]', 'FontSize', fs);
    ylabel(sprintf('$e_{q_%d}$ [rad]', i), 'Interpreter', 'latex', 'FontSize', fs);
    title(jointNames{i}, 'FontSize', fs_title);
    grid on; box on; set(gca, 'FontSize', fs); xlim(xlims);
end
title(tl2, sprintf('[%s] SMC - Errores de Seguimiento Articular', mode_label), ...
      'FontSize', 14, 'FontWeight', 'bold');

% ── Figura 3 — Superficies deslizantes ───────────────────────────────────────
figure(3); clf;
set(gcf, 'Color', 'w', 'Position', [130 130 1100 560]);
tl3 = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for i = 1:4
    nexttile(tl3);
    plot(t_log, S(:,i), '-', 'Color', color_surf, 'LineWidth', lw); hold on;
    yline(0, ':', 'LineWidth', 0.8);
    if strcmp(rho_func, 'sat')   % bordes de la capa limite
        yline( phi, '--k', 'LineWidth', 0.8);
        yline(-phi, '--k', 'LineWidth', 0.8);
    end
    xlabel('Tiempo [s]', 'FontSize', fs);
    ylabel(sprintf('$s_{q_%d}$ [rad/s]', i), 'Interpreter', 'latex', 'FontSize', fs);
    title(jointNames{i}, 'FontSize', fs_title);
    grid on; box on; set(gca, 'FontSize', fs); xlim(xlims);
end
title(tl3, sprintf('[%s] SMC - Superficies Deslizantes', mode_label), ...
      'FontSize', 14, 'FontWeight', 'bold');

% ── Figura 4 — Torques de control ────────────────────────────────────────────
figure(4); clf;
set(gcf, 'Color', 'w', 'Position', [140 140 1100 700]);
for i = 1:4
    subplot(2,2,i);
    plot(t_log, TAU(:,i), '-', 'Color', color_tau, 'LineWidth', lw);
    xlabel('Tiempo [s]', 'FontSize', fs);
    ylabel(sprintf('$\\tau_%d\\;[\\mathrm{N\\cdot m}]$', i), ...
           'Interpreter', 'latex', 'FontSize', fs);
    title(jointNames{i}, 'FontSize', fs_title);
    grid on; box on; set(gca, 'FontSize', fs); xlim(xlims);
end
sgtitle(sprintf('[%s] SMC - Torques de control calculados', mode_label), ...
        'FontSize', 14, 'FontWeight', 'bold');

%% ── Exportacion ──────────────────────────────────────────────────────────────
if EXPORT_FIGS
    output_dir = fullfile(this_dir, 'plots');
    if ~exist(output_dir, 'dir'), mkdir(output_dir); end

    % PNG (raster, 300 dpi)
    exportgraphics(figure(1), fullfile(output_dir, 'tracking_plot_q.png'),     'Resolution', 300);
    exportgraphics(figure(2), fullfile(output_dir, 'tracking_plot_error.png'), 'Resolution', 300);
    exportgraphics(figure(3), fullfile(output_dir, 'surfaces_plot.png'),       'Resolution', 300);
    exportgraphics(figure(4), fullfile(output_dir, 'torques_plot.png'),        'Resolution', 300);

    % EPS (vectorial, 600 dpi)
    exportgraphics(figure(1), fullfile(output_dir, 'tracking_plot_q.eps'),     'ContentType', 'vector', 'Resolution', 600);
    exportgraphics(figure(2), fullfile(output_dir, 'tracking_plot_error.eps'), 'ContentType', 'vector', 'Resolution', 600);
    exportgraphics(figure(3), fullfile(output_dir, 'surfaces_plot.eps'),       'ContentType', 'vector', 'Resolution', 600);
    exportgraphics(figure(4), fullfile(output_dir, 'torques_plot.eps'),        'ContentType', 'vector', 'Resolution', 600);

    % Vista 3D solo en PNG: en EPS vectorial, con las mallas STL, son ~43 MB y ~2.5 min
    if ~isempty(H) && isvalid(H.fig)
        exportgraphics(H.fig, fullfile(output_dir, 'robot3d_final.png'), 'Resolution', 300);
    end
    fprintf('Graficas guardadas en: %s\n', output_dir);
end

%% ═════════════════════════ Funciones locales ═════════════════════════════════

% ── Trayectoria articular deseada ──────────────────────────────────────────────
% ═══════════════════════════════════════════════════════════════════════════
%  SECCION 1 — TRAYECTORIA DE REFERENCIA ARTICULAR  (COMPLETAR)
%
%  Parametro de entrada:
%    t — tiempo actual de simulacion [s]
%
%  Salida esperada:
%    qd   = [q1_des(t);    q2_des(t);   q3_des(t);   q4_des(t)]     [rad]
%    dqd  = [dq1_des(t);   dq2_des(t);  dq3_des(t);  dq4_des(t)]    [rad/s]
%    ddqd = [ddq1_des(t);  ddq2_des(t); ddq3_des(t); ddq4_des(t)]   [rad/s²]
% ═══════════════════════════════════════════════════════════════════════════
function [qd, dqd, ddqd] = trayectoria_deseada(t)
    % -------------------------------------------------------------------
    % COMPLETAR
    % -------------------------------------------------------------------
    w = 1.0;

    qd   = zeros(4,1);
    dqd  = zeros(4,1);
    ddqd = zeros(4,1);
end
% ═══════════════════════════════════════════════════════════════════════════

% ── Funciones de conmutacion ───────────────────────────────────────────────────
% ═══════════════════════════════════════════════════════════════════════════
%  SECCION 3 — FUNCIONES DE CONMUTACION  (COMPLETAR)
%
%  Parametros de entrada:
%    s        — superficie deslizante (4x1)
%    rho_func — 'sat' | 'sign'
%    phi      — capa limite de sat [rad/s]
%
%  Salida esperada:
%    r — funcion de conmutacion, elemento a elemento (4x1)
% ═══════════════════════════════════════════════════════════════════════════
function r = rho_vec(s, rho_func, phi)
    r = zeros(4,1);
    switch rho_func
        case 'sat'
            % COMPLETAR: sat(s/phi)
        case 'sign'
            % COMPLETAR: sign(s)
    end
end
% ═══════════════════════════════════════════════════════════════════════════

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

% ── Cinematica directa analitica ───────────────────
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

% ── Vista 3D del robot: STLs del URDF ──────
function H = build_robot3d(mesh_dir, q0, y_des_path)
    H.fig = figure(5); clf(H.fig);
    set(H.fig, 'Color', 'w', 'Name', 'OpenMANIPULATOR-X — Prelab SMC', ...
        'Position', [80 80 900 700]);
    H.ax = axes(H.fig); hold(H.ax, 'on');
    grid(H.ax, 'on'); box(H.ax, 'on'); axis(H.ax, 'equal');
    xlim(H.ax, [-0.15 0.45]); ylim(H.ax, [-0.30 0.30]); zlim(H.ax, [-0.01 0.3]);
    xlabel(H.ax, 'x [m]'); ylabel(H.ax, 'y [m]'); zlabel(H.ax, 'z [m]');
    view(H.ax, 45, 25);
    title(H.ax, 'Sliding Mode Control articular — OpenMANIPULATOR-X');

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
    y = fkin_local(q);
    addpoints(H.trace, y(1), y(2), y(3));
end
