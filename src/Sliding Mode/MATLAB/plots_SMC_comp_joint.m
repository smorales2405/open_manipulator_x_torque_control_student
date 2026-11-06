%% plots_SMC_comp_joint.m
% Graficas COMPARATIVAS — Control articular (SMC y Feedback Linearization)
% OpenMANIPULATOR-X
%
%   Compara dos o mas ensayos sobre la misma trayectoria articular. Elegir la
%   comparacion con la variable "comparison" y ajustar los test_num de su tabla:
%
%     Actividad 1  'act1_sf'        sign(s) vs sat(s/phi), Gazebo, planta nominal
%     Actividad 2  'act2_int_sim'   sat(s/phi) sin vs con integral, Gazebo, con
%                                   mass_inertia_scale 1.25, damping_scale 1.5 y
%                                   friction_scale 1.5 (los tres a la vez)
%                  'act2_int_real'  sat(s/phi) sin vs con integral, implementacion
%     Actividad 3  'act3_fl_sim'    SMC con integral vs FL, Gazebo, con carga
%                  'act3_fl_real'   SMC con integral vs FL, implementacion, con carga
%     'custom'                      cualquier combinacion (editar su tabla)
%
%   Los test_num de cada ensayo se ajustan en la seccion "Configuracion".
%
%   Figura 1 — Seguimiento de posiciones articulares  q1..q4
%   Figura 2 — Error de seguimiento articular         e_q1..e_q4
%   Figura 3 — Torques de control                     tau1..tau4
%   Figura 4 — Superficies deslizantes                s1..s4  (solo ensayos SMC)
%   Figura 5 — Termino integral                       z1..z4  (solo ensayos con integral)
%   En act3_fl_sim y act3_fl_real solo se generan y guardan las Figuras 1-3.
%
%   Metricas reportadas en consola por ensayo y articulacion:
%     e_max [rad]  |  e_RMS [rad]  |  max|tau| [N·m]  |  tau_RMS [N·m]  |  Sat [%]  |  TV(tau)
%   Se calculan solo en el seguimiento (se descarta la transicion inicial) y
%   en la ventana de tiempo comun a todos los ensayos, para que TV(tau), que
%   crece con la duracion, sea comparable.
%
%   Para comparar con Feedback Linearization, el nodo FL debe seguir la MISMA
%   trayectoria que el SMC, empezando en la misma fase, y correr con el mismo
%   t_run. El script avisa si las referencias de los ensayos no coinciden.
%   En Gazebo, correr el nodo FL con -p friction_ffwd:=true: los nodos SMC
%   siempre compensan la friccion del URDF.
%
% Configurar la seccion "Configuracion" y ejecutar.

clear; clc; close all;

%% ── Configuracion ────────────────────────────────────────────────────────────
comparison  = 'act1_sf';   % 'act1_sf' | 'act2_int_sim' | 'act2_int_real' |
                           % 'act3_fl_sim' | 'act3_fl_real' | 'custom'

EXPORT_FIGS = false;       % true = guardar PNG (300 dpi) y EPS vectorial (600 dpi)

TAU_MAX     = 1.2;         % [N·m] limite de torque (debe coincidir con los nodos)

% Duracion de la transicion inicial de cada nodo [s] (debe coincidir con el
% nodo); esas muestras se descartan
T_START_SMC = 3.0;         % T_TRANS de gz_SMC_joint_node y hw_SMC_joint_node
T_START_FL  = 3.0;         % RAMP_TIME_S de gz_fl_control_node y hw_fl_control_node

% test_num de cada ensayo (el usado al lanzar el nodo)
% Actividad 1 — act1_sf
test_num_sign         = 2;   % sign(s)
test_num_sat          = 1;   % sat(s/phi)
% Actividad 2 — act2_int_sim
test_num_sim_sin_int  = 3;   % SMC sin integral
test_num_sim_con_int  = 4;   % SMC con integral
% Actividad 2 — act2_int_real
test_num_real_sin_int = 1;   % SMC sin integral
test_num_real_con_int = 2;   % SMC con integral
% Actividad 3 — act3_fl_sim
test_num_sim_smc      = 5;   % SMC con integral
test_num_sim_fl       = 1;   % Feedback Linearization
% Actividad 3 — act3_fl_real
test_num_real_smc     = 3;   % SMC con integral
test_num_real_fl      = 1;   % Feedback Linearization

% Tabla de ensayos de cada comparacion. Una fila por ensayo:
%   etiqueta | controlador 'smc'|'fl' | modo 'sim'|'real' |
%   rho 'sat'|'sign' ('' en FL) | integral true|false ([] en FL) | test_num
% extra_figs: generar las Figuras 4 (superficies) y 5 (termino integral)
extra_figs = true;
switch comparison
    case 'act1_sf'
        comp_title = 'sign(s) vs sat(s/\phi)';
        cases = {
            'sign(s)',          'smc', 'sim',  'sign', false, test_num_sign
            'sat(s/\phi)',      'smc', 'sim',  'sat',  false, test_num_sat
        };
    case 'act2_int_sim'
        comp_title = 'Variacion de masa y friccion — sin vs con integral';
        cases = {
            'SMC sin integral', 'smc', 'sim',  'sat',  false, test_num_sim_sin_int
            'SMC con integral', 'smc', 'sim',  'sat',  true,  test_num_sim_con_int
        };
    case 'act2_int_real'
        comp_title = 'sat(s/\phi) sin vs con integral';
        cases = {
            'SMC sin integral', 'smc', 'real', 'sat',  false, test_num_real_sin_int
            'SMC con integral', 'smc', 'real', 'sat',  true,  test_num_real_con_int
        };
    case 'act3_fl_sim'
        comp_title = 'Con carga — SMC con integral vs FL';
        cases = {
            'SMC con integral', 'smc', 'sim',  'sat',  true,  test_num_sim_smc
            'FL',               'fl',  'sim',  '',     [],    test_num_sim_fl
        };
        extra_figs = false;   % solo Figuras 1-3
    case 'act3_fl_real'
        comp_title = 'Con carga — SMC con integral vs FL';
        cases = {
            'SMC con integral', 'smc', 'real', 'sat',  true,  test_num_real_smc
            'FL',               'fl',  'real', '',     [],    test_num_real_fl
        };
        extra_figs = false;   % solo Figuras 1-3
    case 'custom'
        comp_title = 'Comparacion';
        cases = {
            'Ensayo A',         'smc', 'sim',  'sat',  false, 1
            'Ensayo B',         'smc', 'sim',  'sat',  true,  2
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
              'z', {}, 'z_on', {});
for k = 1:n_cases
    [lab, ctrl, md, rho, integ, tnum] = cases{k, :};
    if ~isfield(node_pre, md)
        error('Ensayo "%s": modo ''%s'' invalido (''sim'' | ''real'').', lab, md);
    end
    switch ctrl
        case 'smc'
            fpath = fullfile(pkg_dir, 'data', 'smc', md, 'act1', ...
                             sprintf('%s_smc_joint_%s_%d.csv', node_pre.(md), rho, tnum));
            t0 = T_START_SMC;
        case 'fl'
            fpath = fullfile(pkg_dir, 'data', 'feedback_linearization', md, 'act1', ...
                             sprintf('%s_fl_data_%d.csv', node_pre.(md), tnum));
            t0 = T_START_FL;
        otherwise
            error('Ensayo "%s": controlador ''%s'' invalido (''smc'' | ''fl'').', lab, ctrl);
    end
    if ~isfile(fpath)
        error('Archivo no encontrado:\n  %s\nVerificar la tabla de "%s".', fpath, comparison);
    end
    Tk = readtable(fpath);
    Tk = Tk(Tk.t >= t0, :);   % descarta la transicion inicial
    if height(Tk) < 2
        error('"%s": no hay muestras con t >= %.1f s en\n  %s', lab, t0, fpath);
    end
    vars = Tk.Properties.VariableNames;

    d.label = lab;
    d.ctrl  = ctrl;
    d.mode  = md;
    d.test  = tnum;
    d.t     = Tk.t - t0;   % tiempo de seguimiento (0 = inicio de la trayectoria)
    d.q     = [Tk.q1,     Tk.q2,     Tk.q3,     Tk.q4    ];
    d.q_des = [Tk.q1_des, Tk.q2_des, Tk.q3_des, Tk.q4_des];
    d.e_q   = d.q - d.q_des;
    d.tau   = [Tk.tau1,   Tk.tau2,   Tk.tau3,   Tk.tau4  ];
    if all(ismember({'s1','s2','s3','s4'}, vars))
        d.s = [Tk.s1, Tk.s2, Tk.s3, Tk.s4];
    else
        d.s = [];
    end
    % z1..z4 vale cero con integral:=false y no existe en los CSV anteriores
    % a ese parametro
    if all(ismember({'z1','z2','z3','z4'}, vars))
        d.z = [Tk.z1, Tk.z2, Tk.z3, Tk.z4];
    else
        d.z = zeros(height(Tk), 4);
    end
    d.z_on = any(d.z(:) ~= 0);

    % El nombre del CSV no indica si el ensayo uso integral: se verifica con z
    if strcmp(ctrl, 'smc') && ~isempty(integ) && logical(integ) ~= d.z_on
        if integ
            warning('"%s" (test %d) deberia tener integral, pero z = 0: revisar el test_num.', lab, tnum);
        else
            warning('"%s" (test %d) no deberia tener integral, pero z ~= 0: revisar el test_num.', lab, tnum);
        end
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
    if ~isempty(data(k).s)
        data(k).s = data(k).s(m, :);
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
fprintf(' Comparacion articular — %s  [%s]  (seguimiento: %.1f s)\n', ...
        comp_title, mode_label, t_end);
fprintf('%s\n', repmat('═', 1, 88));
for k = 1:n_cases
    sat_k = abs(data(k).tau) >= 0.99 * TAU_MAX;   % criterio de la guia
    fprintf('\n  %s  (%s, test=%d)\n', data(k).label, upper(data(k).ctrl), data(k).test);
    fprintf('  %-6s  %-12s  %-12s  %-14s  %-14s  %-8s  %-10s\n', ...
            'Joint','e_max[rad]','e_RMS[rad]','max|tau|[N·m]','tau_RMS[N·m]','Sat[%]','TV(tau)');
    fprintf('  %s\n', repmat('-', 1, 84));
    for i = 1:4
        fprintf('  q%-5d  %12.5f  %12.5f  %14.4f  %14.4f  %8.2f  %10.4f\n', i, ...
                max(abs(data(k).e_q(:,i))), sqrt(mean(data(k).e_q(:,i).^2)), ...
                max(abs(data(k).tau(:,i))), sqrt(mean(data(k).tau(:,i).^2)), ...
                100 * mean(sat_k(:,i)), sum(abs(diff(data(k).tau(:,i)))));
    end
    if data(k).z_on
        fprintf('  z media [rad/s2] = [%.2f %.2f %.2f %.2f]  (z converge a menos la perturbacion)\n', ...
                mean(data(k).z));
    end
end

fprintf('\n  Resumen e_RMS [mrad]\n');
fprintf('  %-24s %9s %9s %9s %9s\n', 'Ensayo', 'q1', 'q2', 'q3', 'q4');
for k = 1:n_cases
    fprintf('  %-24s %9.2f %9.2f %9.2f %9.2f\n', data(k).label, ...
            1e3 * sqrt(mean(data(k).e_q.^2, 1)));
end
fprintf('%s\n\n', repmat('═', 1, 88));

%% ── Estilo comun ─────────────────────────────────────────────────────────────
sty.lw       = 1.5;
sty.fs       = 11;
sty.fs_title = 14;
sty.t_end    = t_end;
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
smc_k     = find(arrayfun(@(d) ~isempty(d.s), data));
z_k       = find([data.z_on]);

%% ── Figuras ──────────────────────────────────────────────────────────────────
comp_figure(1, [50 620 1100 580], data, all_k, 'q', ...
    @(i) sprintf('$q_%d$ [rad]', i), title_str, sty, 'q_des', ref_match, []);
comp_figure(2, [70 390 1100 580], data, all_k, 'e_q', ...
    @(i) sprintf('$e_{q_%d}$ [rad]', i), [title_str '  — Errores'], sty, '', true, []);
comp_figure(3, [90 160 1100 580], data, all_k, 'tau', ...
    @(i) sprintf('$\\tau_%d\\;[\\mathrm{N{\\cdot}m}]$', i), [title_str '  — Torques'], ...
    sty, '', true, TAU_MAX);
if extra_figs && ~isempty(smc_k)
    comp_figure(4, [110 -80 1100 580], data, smc_k, 's', ...
        @(i) sprintf('$s_{q_%d}$ [rad/s]', i), [title_str '  — Superficies Deslizantes'], ...
        sty, '', true, []);
end
if extra_figs && ~isempty(z_k)
    comp_figure(5, [130 -320 1100 580], data, z_k, 'z', ...
        @(i) sprintf('$z_%d$ [rad/s$^2$]', i), [title_str '  — Termino Integral'], ...
        sty, '', true, []);
end

%% ── Exportacion ──────────────────────────────────────────────────────────────
if EXPORT_FIGS
    output_dir = fullfile(pkg_dir, 'plots', 'smc', mode_dir, 'act1', ['comp_' comparison]);
    if ~exist(output_dir, 'dir'); mkdir(output_dir); end

    fnames = {'plot_comp_q_tracking', 'plot_comp_q_error', 'plot_comp_torques', ...
              'plot_comp_surfaces',   'plot_comp_integral'};
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
% tau_max: dibuja los limites de torque ([] = no).
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
