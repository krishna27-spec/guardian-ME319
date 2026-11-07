% CONTROLLER_TESTS  Time-domain tests of the controls section (Sec. 6 table),
% run on the nonlinear sampled loop: 500 Hz Tustin controller, RK4 plant at
% 1 ms with the voltage held between samples, +/-300 V saturation, anti-windup.
% Synthetic references, no estimator. Figure: figs/C1_controller_tests.png

clear; close all;
here = fileparts(mfilename('fullpath')); addpath(here);
headless_plots();
figdir = fullfile(here, 'figs'); if ~exist(figdir, 'dir'), mkdir(figdir); end
c  = ctrl_params(); ep = est_params();
on = struct('use_ff', true,  'use_aw', true);
noff = struct('use_ff', false, 'use_aw', true);
fprintf('\nController tests (compare with the controls section, Sec. 6)\n');

% --- 1. step of 5 mrad --------------------------------------------------------
A = 5e-3;
R1 = ctrl_loop_sim(@(t) A, @(t) 0, 2, on);
os = 100*(max(R1.theta) - A)/A;
ts = settle_time(R1.t, R1.e, 0.02*A);
fprintf('  step 5 mrad:          overshoot %.1f %%, 2%% settling %.2f s, peak %.0f V\n', os, ts, max(abs(R1.V)));

% --- 2./3. ramp 0.19 rad/s from rest, without and with feed-forward ----------
w = 0.19;
for o = 1:2
    if o == 1, op = noff; lab = 'no feed-forward  '; else, op = on; lab = 'with feed-forward'; end
    R = ctrl_loop_sim(@(t) w*t, @(t) w, 3, op);
    fprintf('  ramp 0.19, %s: steady error %.3f mrad, peak %.1f mrad, inside 0.2 mrad after %.2f s, steady %.0f V, peak %.0f V\n', ...
            lab, 1e3*abs(R.e(end)), 1e3*max(abs(R.e)), settle_time(R.t, R.e, 0.2e-3), R.V(end), max(abs(R.V)));
    if o == 2, R2 = R; end
end

% --- 4. parabola 0.06 rad/s^2, no feed-forward -----------------------------------
ac = 0.06;
R4 = ctrl_loop_sim(@(t) 0.5*ac*t^2, @(t) ac*t, 3, noff);
Ka = c.Kc*c.z_i*c.N*c.Kt/(c.Rm*c.b + c.N^2*c.Kt*c.Ke);
fprintf('  parabola 0.06, no FF:  steady error %.3f mrad (theory a/Ka = %.3f mrad)\n', 1e3*R4.e(end), 1e3*ac/Ka);

% --- 5. slew of 0.5 rad at 300 V, with and without anti-windup -------------------
for o = 1:2
    if o == 1, op = on; lab = 'with anti-windup   '; else, op = struct('use_ff', true, 'use_aw', false); lab = 'without anti-windup'; end
    R = ctrl_loop_sim(@(t) 0.5, @(t) 0, 8, op);
    ts2 = settle_time(R.t, R.e, 2e-3);
    if isnan(ts2), tstr = 'never (sustained windup oscillation)'; else, tstr = sprintf('%.2f s', ts2); end
    fprintf('  slew 0.5 rad, %s: overshoot %.1f %%, inside 2 mrad after %s, peak current %.0f A\n', ...
            lab, 100*(max(R.theta) - 0.5)/0.5, tstr, max(abs(R.i)));
    if o == 1, R5 = R; end
end

% --- 6. ramp into the 84.7 deg reference clamp, with feed-forward ---------------
th0 = 75*pi/180; lim = ep.theta_ref_lim(2);
rf = @(t) min(th0 + w*t, lim);
rd = @(t) w*(th0 + w*t < lim);
R6 = ctrl_loop_sim(rf, rd, 2.5, on, [th0; 0; 0]);
fprintf('  ramp into the clamp:   overshoot past 84.7 deg = %.2f deg (stop at 85 deg never reached: %d)\n', ...
        rad2deg(max(R6.theta) - lim), max(R6.theta) < c.theta_lim(2));

% --- figure ------------------------------------------------------------------------
figure('visible', 'off', 'position', [0 0 1000 700]);
subplot(2,2,1); plot(R1.t, 1e3*R1.ref, 'k--', R1.t, 1e3*R1.theta, 'b'); grid on;
title(sprintf('Step 5 mrad: %.1f%% overshoot, %.2f s settling', os, ts)); xlabel('t [s]'); ylabel('\theta [mrad]');
subplot(2,2,2); plot(R2.t, 1e3*R2.e, 'b'); grid on; ylim([-1 5]);
title('Ramp 0.19 rad/s with feed-forward: error'); xlabel('t [s]'); ylabel('e [mrad]');
subplot(2,2,3); plot(R4.t, 1e3*R4.e, 'b', R4.t, 1e3*ac/Ka*ones(size(R4.t)), 'r--'); grid on;
title('Parabola 0.06 rad/s^2, no FF: error vs a/K_a'); xlabel('t [s]'); ylabel('e [mrad]');
subplot(2,2,4); ax = plotyy(R5.t, R5.theta, R5.t, R5.V); grid on;
ylabel(ax(1), '\theta [rad]'); ylabel(ax(2), 'V [V]');
title('Slew 0.5 rad at 300 V, anti-windup'); xlabel('t [s]');
print(fullfile(figdir, 'C1_controller_tests.png'), '-dpng', '-r110');
fprintf('Figure saved: figs/C1_controller_tests.png\n');
