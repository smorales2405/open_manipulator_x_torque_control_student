%% plots_MRAC_comp_joint.m
% Graficas COMPARATIVAS — Control articular MRAC y SMC
% OpenMANIPULATOR-X
%
%   Compara dos o mas ensayos sobre la misma trayectoria articular (la de los
%   nodos SMC, que tambien siguen los nodos MRAC). Elegir la comparacion con la
%   variable "comparison" y ajustar los test_num en la seccion "Configuracion":
%
%     'act1_adapt_sim'   MRAC sin vs con adaptacion, Gazebo
%     'act1_adapt_real'  MRAC sin vs con adaptacion, implementacion
%     'act2_smc_sim'     SMC con integral vs MRAC adaptativo, Gazebo con la
%                        planta modificada (las mismas escalas en ambos)
%     'act2_smc_real'    SMC con integral vs MRAC adaptativo, implementacion
%     'act3_smc_sim'     SMC con integral vs MRAC adaptativo, Gazebo con carga
%     'act3_smc_real'    SMC con integral vs MRAC adaptativo, implementacion
%                        con carga
%     'custom'           cualquier combinacion (editar su tabla)
%   El MRAC sin adaptacion (adaptive:=false) usa la misma ley que el SMC sin
%   integral; el adaptativo reemplaza el integral por el feedforward Y*a_hat.
%
%   Figura 1 — Seguimiento de posiciones articulares  q1..q4
%   Figura 2 — Error de seguimiento articular         e_q1..e_q4
%   Figura 3 — Torques de control                     tau1..tau4
%   Figura 4 — Superficies                            s1..s4
%   Figura 5 — Friccion de Coulomb estimada Fc_i      (solo ensayos MRAC adaptativos)
%   Figura 6 — Termino integral z1..z4                (solo ensayos SMC con integral)
%
%   Metricas reportadas en consola por ensayo y articulacion:
%     e_max [rad]  |  e_RMS [rad]  |  max|tau| [N·m]  |  tau_RMS [N·m]  |  Sat [%]  |  TV(tau)
%   y los parametros finales de los ensayos MRAC. Se calculan solo en el
%   seguimiento (se descarta la transicion inicial) y en la ventana de tiempo
%   comun a todos los ensayos, para que TV(tau), que crece con la duracion,
%   sea comparable. Con with_load = true (Actividad 3, t_run de 40 s) las
%   metricas se calculan desde T_METRICS_LOAD s de seguimiento: en la primera
%   parte el MRAC todavia esta aprendiendo la carga. Las graficas muestran el
%   ensayo completo.
%
% Configurar la seccion "Configuracion" y ejecutar.

clear; clc; close all;

%% ── Configuracion ────────────────────────────────────────────────────────────
comparison  = 'act1_adapt_sim';   % 'act1_adapt_sim' | 'act1_adapt_real' |
                                % 'act2_smc_sim' | 'act2_smc_real' |
                                % 'act3_smc_sim' | 'act3_smc_real' | 'custom'

with_load   = false;       % true = ensayos con carga en el efector (Actividad 3)

EXPORT_FIGS = false;       % true = guardar PNG (300 dpi) y EPS vectorial (600 dpi)

TAU_MAX     = 1.2;         % [N·m] limite de torque (debe coincidir con los nodos)

% Duracion de la transicion inicial de los nodos [s] (T_TRANS de los nodos
% SMC y MRAC); esas muestras se descartan
T_START     = 3.0;

% Con with_load = true, las metricas se calculan desde este tiempo de
% seguimiento [s] (sin carga, desde 0)
T_METRICS_LOAD = 20.0;

% test_num de cada ensayo (el usado al lanzar el nodo)
% act1_adapt_sim
test_num_sim_fixed     = 1;   % MRAC sin adaptacion (adaptive:=false)
test_num_sim_adapt     = 2;   % MRAC adaptativo
% act1_adapt_real
test_num_real_fixed    = 1;   % MRAC sin adaptacion (adaptive:=false)
test_num_real_adapt    = 2;   % MRAC adaptativo
% act2_smc_sim
test_num_sim2_smc      = 4;   % SMC con integral (gz_smc_joint_sat_<N>.csv)
test_num_sim2_mrac     = 3;   % MRAC adaptativo
% act2_smc_real
test_num_real2_smc     = 2;   % SMC con integral (hw_smc_joint_sat_<N>.csv)
test_num_real2_mrac    = 2;   % MRAC adaptativo
% act3_smc_sim
test_num_sim3_smc      = 5;   % SMC con integral, con carga
test_num_sim3_mrac     = 4;   % MRAC adaptativo, con carga
% act3_smc_real
test_num_real3_smc     = 3;   % SMC con integral, con carga
test_num_real3_mrac    = 3;   % MRAC adaptativo, con carga

% Tabla de ensayos de cada comparacion. Una fila por ensayo:
%   etiqueta | controlador 'mrac'|'smc'|'fl' | modo 'sim'|'real' |
%   variante: MRAC 'adaptive'|'fixed',
%             SMC  'sat'|'sign' (la funcion del CSV), FL '' | test_num
switch comparison
    case 'act1_adapt_sim'
        comp_title = 'MRAC sin vs con adaptacion';
        cases = {
            'MRAC sin adaptacion', 'mrac', 'sim',  'fixed',    test_num_sim_fixed
            'MRAC adaptativo',     'mrac', 'sim',  'adaptive', test_num_sim_adapt
        };
    case 'act1_adapt_real'
        comp_title = 'MRAC sin vs con adaptacion';
        cases = {
            'MRAC sin adaptacion', 'mrac', 'real', 'fixed',    test_num_real_fixed
            'MRAC adaptativo',     'mrac', 'real', 'adaptive', test_num_real_adapt
        };
    case 'act2_smc_sim'
        comp_title = 'Planta modificada — SMC con integral vs MRAC';
        cases = {
            'SMC con integral',    'smc',  'sim',  'sat',      test_num_sim2_smc
            'MRAC adaptativo',     'mrac', 'sim',  'adaptive', test_num_sim2_mrac
        };
    case 'act2_smc_real'
        comp_title = 'SMC con integral vs MRAC';
        cases = {
            'SMC con integral',    'smc',  'real', 'sat',      test_num_real2_smc
            'MRAC adaptativo',     'mrac', 'real', 'adaptive', test_num_real2_mrac
        };
    case 'act3_smc_sim'
        comp_title = 'Con carga — SMC con integral vs MRAC';
        cases = {
            'SMC con integral',    'smc',  'sim',  'sat',      test_num_sim3_smc
            'MRAC adaptativo',     'mrac', 'sim',  'adaptive', test_num_sim3_mrac
        };
    case 'act3_smc_real'
        comp_title = 'Con carga — SMC con integral vs MRAC';
        cases = {
            'SMC con integral',    'smc',  'real', 'sat',      test_num_real3_smc
            'MRAC adaptativo',     'mrac', 'real', 'adaptive', test_num_real3_mrac
        };
    case 'custom'
        comp_title = 'Comparacion';
        cases = {
            'Ensayo A',            'mrac', 'sim',  'fixed',    1
            'Ensayo B',            'mrac', 'sim',  'adaptive', 2
        };
    otherwise
        error('comparison desconocida: ''%s''.', comparison);
end

% Directorio raiz del paquete ROS 2
pkg_dir = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));   % raiz del paquete (relativa a este script)

%% ── Carga de datos ───────────────────────────────────────────────────────────
n_cases  = size(cases, 1);
node_pre = struct('sim', 'gz', 'real', 'hw');

data = struct('label', {}, 'ctrl', {}, 'mode', {}, 'test', {}, 't', {}, ...
              'q', {}, 'q_des', {}, 'e_q', {}, 'tau', {}, 's', {}, ...
              'z', {}, 'z_on', {}, 'fc', {}, 'par', {}, 'adapt_on', {});
for k = 1:n_cases
    [lab, ctrl, md, variant, tnum] = cases{k, :};
    if ~isfield(node_pre, md)
        error('Ensayo "%s": modo ''%s'' invalido (''sim'' | ''real'').', lab, md);
    end
    switch ctrl
        case 'mrac'
            fpath = fullfile(pkg_dir, 'data', 'mrac', md, 'mrac12p', ...
                             sprintf('%s_mrac_joint_12p_%s_%d.csv', node_pre.(md), variant, tnum));
        case 'smc'
            fpath = fullfile(pkg_dir, 'data', 'smc', md, 'act1', ...
                             sprintf('%s_smc_joint_%s_%d.csv', node_pre.(md), variant, tnum));
        case 'fl'
            fpath = fullfile(pkg_dir, 'data', 'feedback_linearization', md, 'act1', ...
                             sprintf('%s_fl_data_%d.csv', node_pre.(md), tnum));
        otherwise
            error('Ensayo "%s": controlador ''%s'' invalido (''mrac'' | ''smc'' | ''fl'').', lab, ctrl);
    end
    if ~isfile(fpath)
        error('Archivo no encontrado:\n  %s\nVerificar la tabla de "%s".', fpath, comparison);
    end
    Tk = readtable(fpath);
    Tk = Tk(Tk.t >= T_START, :);   % descarta la transicion inicial
    if height(Tk) < 2
        error('"%s": no hay muestras con t >= %.1f s en\n  %s', lab, T_START, fpath);
    end
    vars = Tk.Properties.VariableNames;

    d.label = lab;
    d.ctrl  = ctrl;
    d.mode  = md;
    d.test  = tnum;
    d.t     = Tk.t - T_START;   % tiempo de seguimiento (0 = inicio de la trayectoria)
    d.q     = [Tk.q1,     Tk.q2,     Tk.q3,     Tk.q4    ];
    d.q_des = [Tk.q1_des, Tk.q2_des, Tk.q3_des, Tk.q4_des];
    d.e_q   = d.q - d.q_des;
    d.tau   = [Tk.tau1,   Tk.tau2,   Tk.tau3,   Tk.tau4  ];
    if all(ismember({'s1','s2','s3','s4'}, vars))
        d.s = [Tk.s1, Tk.s2, Tk.s3, Tk.s4];
    else
        d.s = [];
    end
    if all(ismember({'z1','z2','z3','z4'}, vars))
        d.z = [Tk.z1, Tk.z2, Tk.z3, Tk.z4];
    else
        d.z = zeros(height(Tk), 4);
    end
    d.z_on = any(d.z(:) ~= 0);
    if strcmp(ctrl, 'mrac')
        d.fc  = [Tk.fc1_hat, Tk.fc2_hat, Tk.fc3_hat, Tk.fc4_hat];
        d.par = [Tk.a1_hat, Tk.a2_hat, Tk.a3_hat, Tk.dm_hat, Tk.dmcx_hat, Tk.fv1_hat];
        d.adapt_on = startsWith(variant, 'adaptive');
    else
        d.fc  = zeros(height(Tk), 4);
        d.par = [];
        d.adapt_on = false;
    end

    data(k) = d;
    fprintf('[%s | %s | test=%d]  %s  (%d muestras de seguimiento, %.1f s)\n', ...
            md, ctrl, tnum, lab, height(Tk), d.t(end));
end

%% ── Ventana comun y verificacion de referencias ─────────────────────────────
durs  = arrayfun(@(d) d.t(end), data);
t_end = min(durs);
if max(durs) - t_end > 0.5
    warning('Los ensayos duran distinto (%s s): se comparan los primeros %.1f s de seguimiento.', ...
            strtrim(sprintf('%.1f ', durs)), t_end);
end
for k = 1:n_cases
    m = data(k).t <= t_end + 1e-9;
    data(k).t     = data(k).t(m);
    data(k).q     = data(k).q(m, :);
    data(k).q_des = data(k).q_des(m, :);
    data(k).e_q   = data(k).e_q(m, :);
    data(k).tau   = data(k).tau(m, :);
    data(k).z     = data(k).z(m, :);
    data(k).fc    = data(k).fc(m, :);
    if ~isempty(data(k).s)
        data(k).s = data(k).s(m, :);
    end
    if ~isempty(data(k).par)
        data(k).par = data(k).par(m, :);
    end
end

ref_match = true;
for k = 2:n_cases
    qd_k = interp1(data(k).t, data(k).q_des, data(1).t, 'linear', NaN);
    dif  = max(abs(qd_k - data(1).q_des), [], 1, 'omitnan');
    if any(dif > 1e-3)
        ref_match = false;
        warning(['La referencia de "%s" no coincide con la de "%s" ' ...
                 '(max |diferencia| = [%s] rad). Los nodos deben seguir la misma ' ...
                 'trayectoria desde la misma fase; se grafica la referencia de cada ensayo.'], ...
                data(k).label, data(1).label, strtrim(sprintf('%.3f ', dif)));
    end
end

% Ventana de las metricas: con carga se descarta la primera parte del
% seguimiento, en la que el MRAC todavia esta aprendiendo la carga
if startsWith(comparison, 'act3') ~= with_load
    warning('comparison = ''%s'' con with_load = %d: la Actividad 3 es la de carga.', ...
            comparison, with_load);
end
if with_load
    t_metrics = T_METRICS_LOAD;
else
    t_metrics = 0.0;
end
if t_end <= t_metrics
    error('Los ensayos duran %.1f s de seguimiento: no alcanzan t_metrics = %.1f s.', ...
          t_end, t_metrics);
end

%% ── Metricas comparativas ────────────────────────────────────────────────────
modes = unique(cases(:, 3));
if numel(modes) > 1
    mode_dir   = 'mixed';
    mode_label = 'Simulacion e implementacion';
elseif strcmp(modes{1}, 'sim')
    mode_dir   = 'sim';
    mode_label = 'Simulacion';
else
    mode_dir   = 'real';
    mode_label = 'Implementacion';
end

fprintf('\n%s\n', repmat('═', 1, 88));
fprintf(' Comparacion articular — %s  [%s]\n', comp_title, mode_label);
fprintf(' Metricas en t = %.1f a %.1f s de seguimiento\n', t_metrics, t_end);
fprintf('%s\n', repmat('═', 1, 88));
for k = 1:n_cases
    mk    = data(k).t >= t_metrics;
    e_k   = data(k).e_q(mk, :);
    tau_k = data(k).tau(mk, :);
    sat_k = abs(tau_k) >= 0.99 * TAU_MAX;   % criterio de la guia
    fprintf('\n  %s  (%s, test=%d)\n', data(k).label, upper(data(k).ctrl), data(k).test);
    fprintf('  %-6s  %-12s  %-12s  %-14s  %-14s  %-8s  %-10s\n', ...
            'Joint','e_max[rad]','e_RMS[rad]','max|tau|[N·m]','tau_RMS[N·m]','Sat[%]','TV(tau)');
    fprintf('  %s\n', repmat('-', 1, 84));
    for i = 1:4
        fprintf('  q%-5d  %12.5f  %12.5f  %14.4f  %14.4f  %8.2f  %10.4f\n', i, ...
                max(abs(e_k(:,i))), sqrt(mean(e_k(:,i).^2)), ...
                max(abs(tau_k(:,i))), sqrt(mean(tau_k(:,i).^2)), ...
                100 * mean(sat_k(:,i)), sum(abs(diff(tau_k(:,i)))));
    end
    if data(k).z_on
        fprintf('  z media [rad/s2] = [%.2f %.2f %.2f %.2f]\n', mean(data(k).z(mk, :)));
    end
    if strcmp(data(k).ctrl, 'mrac')
        p_end = data(k).par(end, :);
        fprintf(['  parametros finales: alpha = [%.3f %.3f %.3f]  dm = %.3f kg  ' ...
                 'dmcx = %.4f kg·m  Fv1 = %.4f  Fc = [%.4f %.4f %.4f %.4f]\n'], ...
                p_end(1:3), p_end(4), p_end(5), p_end(6), data(k).fc(end, :));
    end
end

fprintf('\n  Resumen e_RMS [mrad]\n');
fprintf('  %-24s %9s %9s %9s %9s\n', 'Ensayo', 'q1', 'q2', 'q3', 'q4');
for k = 1:n_cases
    mk = data(k).t >= t_metrics;
    fprintf('  %-24s %9.2f %9.2f %9.2f %9.2f\n', data(k).label, ...
            1e3 * sqrt(mean(data(k).e_q(mk, :).^2, 1)));
end
fprintf('%s\n\n', repmat('═', 1, 88));

%% ── Estilo comun ─────────────────────────────────────────────────────────────
sty.lw       = 1.5;
sty.fs       = 11;
sty.fs_title = 14;
sty.t_end    = t_end;
sty.t_mark   = 0;           % sin linea de inicio de metricas (solo en la de errores)
sty.ref      = [0.4 0.4 0.4];   % gris — referencia
palette = [0.0000 0.4470 0.7410;    % azul
           0.8500 0.3250 0.0980;    % naranja
           0.4660 0.6740 0.1880;    % verde
           0.4940 0.1840 0.5560;    % violeta
           0.3010 0.7450 0.9330;    % celeste
           0.6350 0.0780 0.1840];   % granate
if n_cases > size(palette, 1)
    palette = lines(n_cases);
end
sty.colors = palette;

title_str = sprintf('Comparacion articular — %s  [%s]', comp_title, mode_label);
all_k     = 1:n_cases;
s_k       = find(arrayfun(@(d) ~isempty(d.s), data));
fc_k      = find([data.adapt_on]);
z_k       = find([data.z_on]);

%% ── Figuras ──────────────────────────────────────────────────────────────────
comp_figure(1, [50 620 1100 580], data, all_k, 'q', ...
    @(i) sprintf('$q_%d$ [rad]', i), title_str, sty, 'q_des', ref_match, []);
sty_e        = sty;
sty_e.t_mark = t_metrics;
comp_figure(2, [70 390 1100 580], data, all_k, 'e_q', ...
    @(i) sprintf('$e_{q_%d}$ [rad]', i), [title_str '  — Errores'], sty_e, '', true, []);
comp_figure(3, [90 160 1100 580], data, all_k, 'tau', ...
    @(i) sprintf('$\\tau_%d\\;[\\mathrm{N{\\cdot}m}]$', i), [title_str '  — Torques'], ...
    sty, '', true, TAU_MAX);
if ~isempty(s_k)
    comp_figure(4, [110 -80 1100 580], data, s_k, 's', ...
        @(i) sprintf('$s_{%d}$ [rad/s]', i), [title_str '  — Superficies'], ...
        sty, '', true, []);
end
if ~isempty(fc_k)
    comp_figure(5, [130 -320 1100 580], data, fc_k, 'fc', ...
        @(i) sprintf('$\\hat{F}_{c,%d}$ [N$\\cdot$m]', i), ...
        [title_str '  — Friccion de Coulomb estimada (MRAC)'], sty, '', true, []);
end
if ~isempty(z_k)
    comp_figure(6, [150 -560 1100 580], data, z_k, 'z', ...
        @(i) sprintf('$z_%d$ [rad/s$^2$]', i), [title_str '  — Termino Integral (SMC)'], ...
        sty, '', true, []);
end

%% ── Exportacion ──────────────────────────────────────────────────────────────
if EXPORT_FIGS
    output_dir = fullfile(pkg_dir, 'plots', 'mrac', mode_dir, 'mrac12p', ['comp_' comparison]);
    if ~exist(output_dir, 'dir'); mkdir(output_dir); end

    fnames = {'plot_comp_q_tracking', 'plot_comp_q_error', 'plot_comp_torques', ...
              'plot_comp_surfaces',   'plot_comp_fc_hat',  'plot_comp_integral'};
    for f = 1:numel(fnames)
        if ishghandle(f)
            exportgraphics(figure(f), fullfile(output_dir, [fnames{f} '.png']), 'Resolution', 300);
            exportgraphics(figure(f), fullfile(output_dir, [fnames{f} '.eps']), ...
                           'ContentType', 'vector', 'Resolution', 600);
        end
    end
    fprintf('Graficas guardadas en:\n  %s\n', output_dir);
end

%% ── Funciones locales ────────────────────────────────────────────────────────
function comp_figure(fig_num, pos, data, idx, field, ylab, ttl, sty, ref_field, ref_match, tau_max)
% Figura 2x2 (una celda por articulacion) con los ensayos idx superpuestos.
% ref_field: campo de referencia a graficar ('' = ninguna). Si las referencias
% no coinciden se grafica la de cada ensayo, punteada y en su color.
% tau_max: dibuja los limites de torque ([] = no). sty.t_mark > 0 dibuja una
% linea vertical en ese tiempo (inicio de las metricas).
    figure(fig_num); clf;
    set(gcf, 'Color', 'w', 'Position', pos);
    tl = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

    jointNames = {'Articulacion 1', 'Articulacion 2', 'Articulacion 3', 'Articulacion 4'};
    h_leg = gobjects(0);
    names = {};
    for i = 1:4
        nexttile(tl);
        if isempty(ref_field)
            yline(0, ':', 'LineWidth', 0.9, 'Color', [0.5 0.5 0.5]);
        end
        hold on;
        if ~isempty(tau_max)
            yline( tau_max, '--k', 'LineWidth', 1.0, 'Label', sprintf('+%.1f N·m', tau_max));
            yline(-tau_max, '--k', 'LineWidth', 1.0, 'Label', sprintf('\x2212%.1f N·m', tau_max));
        end
        if sty.t_mark > 0
            xline(sty.t_mark, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 0.9, ...
                  'Label', 'inicio de metricas', 'HandleVisibility', 'off');
        end
        if ~isempty(ref_field)
            if ref_match
                hr = plot(data(idx(1)).t, data(idx(1)).(ref_field)(:,i), '-', ...
                          'Color', sty.ref, 'LineWidth', 1.2);
            else
                for k = idx
                    hr_k = plot(data(k).t, data(k).(ref_field)(:,i), ':', ...
                                'Color', sty.colors(k,:), 'LineWidth', 1.2);
                    if k == idx(1); hr = hr_k; end
                end
            end
            if i == 1
                h_leg(end+1) = hr;
                if ref_match
                    names{end+1} = 'Referencia';
                else
                    names{end+1} = 'Referencia de cada ensayo';
                end
            end
        end
        for k = idx
            hk = plot(data(k).t, data(k).(field)(:,i), '-', ...
                      'Color', sty.colors(k,:), 'LineWidth', sty.lw);
            if i == 1; h_leg(end+1) = hk; names{end+1} = data(k).label; end
        end
        xlabel('Tiempo [s]', 'FontSize', sty.fs);
        ylabel(ylab(i), 'Interpreter', 'latex', 'FontSize', sty.fs);
        title(jointNames{i}, 'FontSize', sty.fs);
        if ~isempty(tau_max)
            ylim([-tau_max * 1.25, tau_max * 1.25]);
        end
        grid on; box on; set(gca, 'FontSize', sty.fs);
        xlim([0, sty.t_end]);
    end

    lgd = legend(nexttile(tl, 1), h_leg, names, ...
                 'Orientation', 'horizontal', 'FontSize', sty.fs, 'Interpreter', 'tex');
    lgd.Layout.Tile = 'north';
    title(tl, ttl, 'FontSize', sty.fs_title, 'FontWeight', 'bold');
end
