function run_scenarios(N_random)
% RUN_SCENARIOS  Closed-loop results for the paper: Scenarios A, B, C, the
% stress case BC, and N random admissible cases (random manoeuvre + random
% blackout). One frozen design, no retuning between cases.
%   Figures: figs/S_<case>.png (one per standard scenario), figs/S_random.png
%   Table:   printed, and saved to figs/results.csv
%   run_scenarios       standard scenarios + 20 random cases
%   run_scenarios(N)    with N random cases (0 = standard scenarios only)
% Runtime in Octave: about 15 s per case (MATLAB is several times faster).

if nargin < 1, N_random = 20; end
close all;
here = fileparts(mfilename('fullpath')); addpath(here);
headless_plots();
figdir = fullfile(here, 'figs'); if ~exist(figdir, 'dir'), mkdir(figdir); end
p = est_params();

cases = [{'A', 'B', 'C', 'BC'}, num2cell(1:N_random)];
T = zeros(numel(cases), 11);
fprintf('\n%-5s %6s %5s %10s %9s %9s %8s %10s %8s %8s\n', 'case', 'pulses', 'hits', 'first hit', 'miss avg', 'miss max', 'V peak', 'sat [%]', 'hold err', 'est rms');
fprintf('%-5s %6s %5s %10s %9s %9s %8s %10s %8s %8s\n', '', '', '', '[s]', '[m]', '[m]', '[V]', '', '[mrad]', '[m]');
for c = 1:numel(cases)
    sc = cases{c};
    L = simulate_engagement(sc);
    M = L.metrics;
    if isempty(M.miss), M.miss = NaN; end
    name = L_name(sc);
    T(c,:) = [c, M.n_pulses, M.n_hits, M.t_first_hit, mean(M.miss), max(M.miss), M.V_peak, ...
              100*M.sat_frac, 1e3*M.hold_err_peak, M.est_now_rms, sum(L.bo(:,2) - L.bo(:,1))];
    fprintf('%-5s %6d %5d %10.2f %9.2f %9.2f %8.0f %10.1f %8.2f %8.2f\n', name, M.n_pulses, M.n_hits, ...
            M.t_first_hit, mean(M.miss), max(M.miss), M.V_peak, 100*M.sat_frac, 1e3*M.hold_err_peak, M.est_now_rms);
    if ischar(sc), plot_case(L, name, figdir); end
end
r = 5:numel(cases);
if isempty(r), return; end
fprintf('\nRandom cases: %d/%d intercepted, %d/%d pulses hit, first hit median %.2f s (max %.2f s), mean miss %.2f m\n', ...
        sum(T(r,3) > 0), numel(r), sum(T(r,3)), sum(T(r,2)), median(T(r,4)), max(T(r,4)), mean(T(r,5)));

% --- random-case summary figure ----------------------------------------------
figure('visible', 'off', 'position', [0 0 900 380]);
subplot(1,2,1); bar(T(r,1) - 4, [T(r,3), T(r,2) - T(r,3)], 'stacked'); grid on;
xlabel('random case'); ylabel('pulses'); legend('hit', 'miss', 'location', 'southeast');
title(sprintf('%d random cases: %d/%d pulses hit', numel(r), sum(T(r,3)), sum(T(r,2))));
subplot(1,2,2); plot(T(r,1) - 4, T(r,4), 'bo', T(r,1) - 4, T(r,6), 'r^'); grid on;
xlabel('random case'); legend('first hit time [s]', 'largest miss [m]', 'location', 'northeast');
title('Interception time and accuracy');
print(fullfile(figdir, 'S_random.png'), '-dpng', '-r110');

hdr = 'case,pulses,hits,first_hit_s,miss_mean_m,miss_max_m,V_peak,sat_pct,hold_err_peak_mrad,est_rms_m,blackout_s';
fid = fopen(fullfile(figdir, 'results.csv'), 'w'); fprintf(fid, '%s\n', hdr);
for c = 1:numel(cases)
    fprintf(fid, '%s,%s\n', L_name(cases{c}), strjoin(arrayfun(@(v) sprintf('%.4g', v), T(c,2:end), 'uniformoutput', false), ','));
end
fclose(fid);
fprintf('Figures and figs/results.csv saved in %s\n', figdir);
end

% ------------------------------------------------------------------------
function n = L_name(sc)
if ischar(sc), n = sc; else, n = sprintf('R%d', sc); end
end

function plot_case(L, name, figdir)
p = est_params(); t = L.t;
P = L.pulses; tp = [P.t]; hit = logical([P.hit]);
hold_on = L.mode == 3;
th_true = atan2(L.xt(:,2), L.xt(:,1));
figure('visible', 'off', 'position', [0 0 1100 800]);

subplot(2,2,1); hold on;                       % geometry
plot(L.xt(:,1), L.xt(:,2), 'k', 'linewidth', 1.5);
plot(L.meas(:,1), L.meas(:,2), '.', 'color', [0.6 0.6 0.6], 'markersize', 3);
if ~isempty(P)
    pp = reshape([P.pos], 2, [])';
    plot(pp(hit,1), pp(hit,2), 'go', 'markersize', 8, 'linewidth', 1.5);
    plot(pp(~hit,1), pp(~hit,2), 'rx', 'markersize', 8, 'linewidth', 1.5);
end
plot(0, 0, 'k^', 'markerfacecolor', 'k');
axis equal; grid on; xlabel('x [m]'); ylabel('y [m]');
title(sprintf('Scenario %s: true path, radar dots, pulses (%d/%d hit)', name, sum(hit), numel(P)));

subplot(2,2,2); hold on;                       % angles
yl = rad2deg([min(th_true) max(th_true)]) + [-3 3];
shade(t, hold_on, yl, [0.85 0.93 1]); shade_blackout(L.bo, 1e3);
h = plot(t, rad2deg(th_true), 'k', t, rad2deg(L.theta), 'b', t, rad2deg(L.theta_pred), 'r:');
h(4) = plot(tp, rad2deg([P.theta]), 'g*');
ylim(yl); grid on; xlabel('t [s]'); ylabel('elevation [deg]');
legend(h, {'target angle now', 'turret', 'predicted angle at t+0.5', 'pulses'}, 'location', 'northwest');
title('Angles (blue band = HOLD, grey = blackout)');

subplot(2,2,3); hold on;                       % miss geometry and uncertainty
u = [-sin(L.theta) cos(L.theta)];
dbeam = sum(u .* L.xt, 2);                     % true signed distance of target from beam
shade_blackout(L.bo, 1e3);
plot(t, dbeam, 'b', t, 2*L.sig_now, 'r--', t, -2*L.sig_now, 'r--', t, p.R_eng*ones(size(t)), 'k:', t, -p.R_eng*ones(size(t)), 'k:');
plot(tp, [P.miss] .* sign(interp1(t, dbeam, tp)), 'g*');
ylim([-25 25]); grid on; xlabel('t [s]'); ylabel('[m]');
title('Target distance from beam (blue), +/-2 sigma now (red), +/-R_eng (dotted)', 'interpreter', 'none');

subplot(2,2,4); hold on;                       % voltage and tracking error
plot(t, L.V, 'b'); ylim([-320 320]); grid on; xlabel('t [s]'); ylabel('V_m [V]');
title(sprintf('Motor voltage (saturated %.0f%% of the time)', 100*mean(L.sat)));
print(fullfile(figdir, sprintf('S_%s.png', name)), '-dpng', '-r110');
close;
end

function shade(t, mask, yl, col)
d = diff([0; mask(:); 0]); on = find(d == 1); off = find(d == -1) - 1;
for k = 1:numel(on)
    fill(reshape(t([on(k) off(k) off(k) on(k)]), 1, []), reshape(yl([1 1 2 2]), 1, []), col, 'edgecolor', 'none');
end
end
