% RUN_ESTIMATION_TESTS  Checks for the estimator, predictor and fire logic.
% Runs the estimator open-loop (no turret needed) and saves figures to figs/.
%   T1  scenarios A, B, C, BC: prediction error across the line of sight vs the filter's own 2-sigma
%   T2  Monte Carlo consistency: is the filter's P honest? (NEES, 30 noise seeds)
%   T3  tau_m sensitivity on 20 random admissible cases -> choose and freeze tau_m
%   T4  interface experiment: where must the turret point for the laser to hit?
%       (motivates the lead-and-wait aiming in aim_planner.m)
%   T5  repeated dots: a generator that HOLDS each reading with meas_valid = 1
%       must give exactly the same estimate as one that delivers it once

clear; close all;
here = fileparts(mfilename('fullpath')); addpath(here);
headless_plots();
figdir = fullfile(here, 'figs'); if ~exist(figdir, 'dir'), mkdir(figdir); end
p = est_params();
dts = 1e-2;                 % sim step for the tests [s] (estimator does not depend on it; faster)

%% T1: four scenarios ------------------------------------------------------
scen = {'A', 'B', 'C', 'BC'};
figure('visible', 'off', 'position', [0 0 900 700]);
fprintf('\nT1  prediction error across line of sight at t+%.1f s (t > 1 s)\n', p.T_pred);
fprintf('    scen   rms err [m]   mean 2sigma [m]   inside 2sigma [%%]\n');
for i = 1:numel(scen)
    R = run_case_open_loop(scen{i}, p, dts);
    m = R.t > 1 & isfinite(R.e_pred);
    fprintf('    %-4s   %8.2f      %8.2f          %5.1f\n', scen{i}, sqrt(mean(R.e_pred(m).^2)), ...
            mean(2*R.s_pred(m)), 100*mean(abs(R.e_pred(m)) <= 2*R.s_pred(m)));
    subplot(2, 2, i); hold on;
    shade_blackout(R.blackout, 30);
    plot(R.t, R.e_pred, 'b'); plot(R.t, 2*R.s_pred, 'r--', R.t, -2*R.s_pred, 'r--');
    ylim([-15 15]); grid on; xlabel('t [s]'); ylabel('across-LOS error [m]');
    title(sprintf('Scenario %s: predicted (t+0.5 s) vs truth', scen{i}));
end
print(fullfile(figdir, 'T1_prediction_error.png'), '-dpng', '-r110');

%% T2: Monte Carlo consistency --------------------------------------------
N = 30;
fprintf('\nT2  NEES of the CURRENT position estimate, %d runs, scenario A and B (expected 2)\n', N);
figure('visible', 'off', 'position', [0 0 900 350]);
names = {'A', 'B'};
for j = 1:2
    nees = [];
    for n = 1:N
        R = run_case_open_loop(struct('name', names{j}, 'noise_seed', n), p, dts);
        if isempty(nees), nees = zeros(size(R.nees_pos)); end
        nees = nees + R.nees_pos;
    end
    nees = nees / N;
    lo = 2 - 1.96*sqrt(4/N); hi = 2 + 1.96*sqrt(4/N);     % chi2(2): mean 2, var 4
    m = R.t > 1;
    fprintf('    %s: mean NEES %.2f   (95%% band for the average: [%.2f, %.2f]), inside %.0f%% of time\n', ...
            names{j}, mean(nees(m)), lo, hi, 100*mean(nees(m) >= lo & nees(m) <= hi));
    subplot(1, 2, j); plot(R.t, nees, 'b', R.t, lo*ones(size(R.t)), 'r--', R.t, hi*ones(size(R.t)), 'r--');
    grid on; xlabel('t [s]'); ylabel('average NEES (2 dof)'); ylim([0 8]);
    title(sprintf('Scenario %s: %d-run average NEES', names{j}, N));
end
print(fullfile(figdir, 'T2_consistency.png'), '-dpng', '-r110');

%% T3: tau_m sensitivity ----------------------------------------------------
taus = [0.5 1 2 5 10];
Ncase = 20;
rmsE = zeros(numel(taus), 1); in2 = zeros(numel(taus), 1);
for k = 1:numel(taus)
    pk = p; pk.tau_m = taus(k);
    e_all = []; s_all = [];
    for n = 1:Ncase
        R = run_case_open_loop(n, pk, dts);
        m = R.t > 1 & isfinite(R.e_pred);
        e_all = [e_all; R.e_pred(m)]; s_all = [s_all; R.s_pred(m)]; %#ok<AGROW>
    end
    rmsE(k) = sqrt(mean(e_all.^2)); in2(k) = 100*mean(abs(e_all) <= 2*s_all);
end
fprintf('\nT3  tau_m sensitivity, %d random cases (across-LOS prediction error at t+0.5 s)\n', Ncase);
fprintf('    tau_m [s]   rms err [m]   inside 2sigma [%%]\n');
fprintf('    %6.1f      %8.2f      %6.1f\n', [taus(:) rmsE in2]');
figure('visible', 'off', 'position', [0 0 500 350]);
semilogx(taus, rmsE, 'o-'); grid on; xlabel('\tau_m [s]'); ylabel('rms across-LOS prediction error [m]');
title('Sensitivity to \tau_m (20 random cases)');
print(fullfile(figdir, 'T3_tau_sensitivity.png'), '-dpng', '-r110');

%% T4: where must the turret point? -----------------------------------------
fprintf('\nT4  idealised turrets, scenario B (fire_check + true hit test)\n');
modes = {'points at theta_ref (perfect tracking of the 0.5 s-ahead reference)', ...
         'points at the current estimate theta_now'};
for mode = 1:2
    target_estimator('reset'); fire_check('reset'); clear target_motion_generator
    pulses = 0; hits = 0;
    for t = 0:dts:10
        [r, th, v] = target_motion_generator(t, 'B');
        e = target_estimator(t, r, th, v);
        if ~e.valid, continue; end
        if mode == 1, th_t = e.theta_ref; else, th_t = e.theta_now; end
        f = fire_check(t, e, th_t);
        if f
            pulses = pulses + 1;
            s = target_truth(t, 'B');
            hits = hits + (abs(-sin(th_t)*s(1) + cos(th_t)*s(4)) <= p.R_eng);
        end
    end
    fprintf('    turret %s:\n        pulses %d, hits %d\n', modes{mode}, pulses, hits);
end
s = target_truth((1:0.5:9)', 'B');
lead = abs(atan2(s(:,4)+0.5*s(:,5), s(:,1)+0.5*s(:,2)) - atan2(s(:,4), s(:,1)));
fprintf('    lead angle theta(t+0.5)-theta(t): %.1f to %.1f mrad = %.0f to %.0f m at the target\n', ...
        1e3*min(lead), 1e3*max(lead), min(lead.*hypot(s(:,1),s(:,4))), max(lead.*hypot(s(:,1),s(:,4))));


%% T5: repeated (held) dots --------------------------------------------------
% Same radar dots, delivered two ways at a 1 ms sim step:
%   once:  meas_valid = 1 only on the first call after a sample (our generator)
%   held:  the reading is repeated with meas_valid = 1 on every call (zero-order hold)
% With TRICK 8 the estimator must ignore the repeats and give identical outputs.
fprintf('\nT5  held dots (zero-order-hold generator) vs single delivery, scenario B, 1 ms step\n');
tt5 = (0:1e-3:3)';
out = zeros(numel(tt5), 2, 2);
for mode = 1:2
    target_estimator('reset'); clear target_motion_generator
    rh = NaN; thh = NaN;
    for i = 1:numel(tt5)
        [r, th, v] = target_motion_generator(tt5(i), 'B');
        if mode == 2                                  % hold the last reading, always valid
            if v, rh = r; thh = th; end
            r = rh; th = thh; v = isfinite(rh);
        end
        e = target_estimator(tt5(i), r, th, v);
        if e.valid, out(i, :, mode) = [e.theta_ref, e.sigma_perp_pred]; end
    end
    if mode == 2, nrep = e.n_rep; end
end
d = max(max(abs(out(:,:,1) - out(:,:,2))));
fprintf('    repeated dots ignored: %d,  max |difference| in theta_ref / sigma_pred: %.1e  (expected 0)\n', nrep, d);

fprintf('\nFigures saved in %s\n', figdir);

