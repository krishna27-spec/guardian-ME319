% CONTROLLER_DESIGN_CHECK  Reproduces the analytical numbers of the controls
% section (Sec. 3-5): plant poles, crossover, phase budget, margins
% (continuous and sampled at 500/200/100 Hz), closed-loop poles, anti-windup
% poles. No toolbox needed: frequency responses are evaluated directly.
% Saves a Bode plot of the loop to figs/C0_loop_bode.png.

clear; close all;
here = fileparts(mfilename('fullpath')); addpath(here);
headless_plots();
figdir = fullfile(here, 'figs'); if ~exist(figdir, 'dir'), mkdir(figdir); end
c = ctrl_params();
K = turret_compensator('matrices');

%% plant ---------------------------------------------------------------------
num_p = c.N*c.Kt;
den_p = conv([1 0], conv([c.J_eq c.b], [c.Lm c.Rm]) + [0 0 c.N^2*c.Kt*c.Ke]);
num_p = num_p/den_p(1); den_p = den_p/den_p(1);
pp = roots(den_p);
Kd = c.N*c.Kt/(c.Rm*c.b + c.N^2*c.Kt*c.Ke);
wr = sqrt(den_p(3)); zr = den_p(2)/(2*wr);
fprintf('\nPlant  Gp(s) = %.4g / (s (s^2 + %.4g s + %.5g))\n', num_p, den_p(2), den_p(3));
fprintf('  poles: %s\n', mat2str(pp.', 4));
fprintf('  resonance wn = %.2f rad/s, zeta = %.4f;  low-frequency gain Kd = %.4e rad/(V s)\n', wr, zr, Kd);
fprintf('  J_eq = %.0f kg m^2;  Kv = %.1f V s/rad\n', c.J_eq, c.Kv);

%% continuous loop --------------------------------------------------------------
Gp = @(s) polyval(num_p, s)./polyval(den_p, s);
Gc = @(s) polyval(K.num, s)./polyval(K.den, s);
L  = @(s) Gp(s).*Gc(s);
[wc, pm, w180, gm] = loop_margins(@(w) L(1j*w), 0.1, 400);
fprintf('\nContinuous loop: crossover %.2f rad/s, PM %.1f deg;  -180 deg at %.1f rad/s, GM %.2f dB\n', wc, pm, w180, gm);
s = 1j*wc;
ph_plant = rad2deg(angle(Gp(s)));
ph_pi    = rad2deg(angle((s + c.z_i)/s));
ph_notch = rad2deg(angle((s^2 + 2*c.zeta_z*c.wn*s + c.wn^2)/(s^2 + 2*c.zeta_p*c.wn*s + c.wn^2)));
fprintf('  phase budget at crossover: plant %.1f, PI %.1f, notch %.1f  ->  PM = %.1f deg\n', ...
        ph_plant, ph_pi, ph_notch, 180 + ph_plant + ph_pi + ph_notch);
Ka = c.Kc*c.z_i*Kd;
fprintf('  acceleration constant Ka = Kc z_i Kd = %.1f 1/s^2 -> parabolic error at 0.06 rad/s^2: %.3f mrad\n', Ka, 1e3*0.06/Ka);
nL = conv(num_p*[0 0 0 1], K.num);  dL = conv(den_p, K.den);
nL = [zeros(1, numel(dL) - numel(nL)) nL];
pcl = roots(dL + nL);
fprintf('  closed-loop poles:\n');
for k = 1:numel(pcl)
    if imag(pcl(k)) >= 0
        fprintf('     %8.3f %+8.3fj   (|p| = %.2f, zeta = %.3f)\n', real(pcl(k)), imag(pcl(k)), abs(pcl(k)), -real(pcl(k))/abs(pcl(k)));
    end
end
fprintf('  stable: %d\n', all(real(pcl) < 0));

%% sampled loop: ZOH plant x Tustin controller ----------------------------------
Ap = [0 1 0; 0 -c.b/c.J_eq c.N*c.Kt/c.J_eq; 0 -c.N*c.Ke/c.Lm -c.Rm/c.Lm];
Bp = [0; 0; 1/c.Lm]; Cp = [1 0 0];
fprintf('\nSampled loop (exact ZOH plant, Tustin controller):\n');
for fs = [500 200 100]
    h = 1/fs;
    E = expm([Ap Bp; zeros(1, 4)]*h); Phi = E(1:3, 1:3); Gam = E(1:3, 4);
    Kh = tustin_ss(K.A, K.B, K.C, K.D, h);
    Ld = @(w) arrayfun(@(ww) (Cp/(exp(1j*ww*h)*eye(3) - Phi)*Gam) * ...
                             (Kh.Cd/(exp(1j*ww*h)*eye(3) - Kh.Ad)*Kh.Bd + Kh.Dd), w);
    [wcd, pmd, w180d, gmd] = loop_margins(Ld, 0.1, 0.99*pi/h);
    fprintf('  %3d Hz: crossover %.2f rad/s, PM %.1f deg, GM %.2f dB at %.1f rad/s\n', fs, wcd, pmd, gmd, w180d);
end
pa = eig(K.Ad - K.Bd*K.Cd/K.Dd);
fprintf('\nAnti-windup conditioned dynamics: |z| = %s (stable if < 1)\n', mat2str(abs(pa).', 4));

%% Bode plot of the continuous loop ------------------------------------------------
w = logspace(-1, 3, 800); Lw = L(1j*w);
figure('visible', 'off', 'position', [0 0 800 600]);
subplot(2,1,1); semilogx(w, 20*log10(abs(Lw)), 'b', [wc wc], [-80 80], 'k:'); grid on;
ylabel('|L| [dB]'); ylim([-80 80]); title(sprintf('Loop L = G_p G_c:  w_c = %.1f rad/s, PM = %.1f deg, GM = %.1f dB', wc, pm, gm));
subplot(2,1,2); semilogx(w, rad2deg(unwrap(angle(Lw))), 'b', w, -180*ones(size(w)), 'r--'); grid on;
xlabel('\omega [rad/s]'); ylabel('phase [deg]');
print(fullfile(figdir, 'C0_loop_bode.png'), '-dpng', '-r110');
fprintf('\nFigure saved: figs/C0_loop_bode.png\n');

