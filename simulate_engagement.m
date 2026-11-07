function L = simulate_engagement(scenario, opts)
% SIMULATE_ENGAGEMENT  Full closed-loop engagement: radar generator ->
% turret_controller -> turret_plant, with the assignment's pulse rules and
% hit test. Used by run_scenarios.m and the live viewer guardian_live.m.
%   L = simulate_engagement(scenario)
%   L = simulate_engagement(scenario, opts)
% opts (all optional):
%   dt          simulation step [s]                       (1e-3)
%   T           duration [s]                              (10)
%   x0          turret initial state [theta; thetadot; i] ([0;0;0], horizontal at rest)
%   on_step     function handle stop = f(L, k), called every draw_every steps
%   draw_every  steps between on_step calls               (20)
%
% Evaluator rules implemented here (assignment Sec. 2-3):
%   * a pulse is a 0->1 transition of fire_cmd; holding 1 gives no new pulse
%   * a transition less than t_recharge after the last ISSUED pulse is ignored
%   * hit if the minimum distance from the laser RAY (from the turret along
%     the turret angle at that instant) to the TRUE target is <= R_eng
%
% Output L: time histories (one row per step), pulse list, metrics.

if nargin < 2, opts = struct(); end
dt = getopt(opts, 'dt', 1e-3); T = getopt(opts, 'T', 10);
x  = getopt(opts, 'x0', [0; 0; 0]); x = x(:);
on_step = getopt(opts, 'on_step', []); every = getopt(opts, 'draw_every', 20);
p = est_params();

clear target_motion_generator
turret_controller('reset');
tt = (0:dt:T)'; n = numel(tt);
nanv = NaN(n, 1);
L = struct('t', tt, 'theta', nanv, 'thetadot', nanv, 'i', nanv, 'V', nanv, 'sat', false(n,1), ...
    'ref', nanv, 'refdot', nanv, 'mode', zeros(n,1), 'theta_now', nanv, 'theta_pred', nanv, ...
    'xe', NaN(n,2), 'xp', NaN(n,2), 'xt', NaN(n,2), 'Pe', NaN(n,3), 'sig_now', nanv, 'sig_pred', nanv, ...
    'meas', NaN(n,2), 'valid', false(n,1), 'blackout', false(n,1), 'fire_ok', false(n,1), ...
    'scenario', scenario, 'k', 0);
L.pulses = struct('t', {}, 'theta', {}, 'miss', {}, 'hit', {}, 'pos', {});
L.ignored = 0;
last_cmd = 0; last_issued = -Inf;
[~, info0] = target_truth(0, scenario); bo = info0.blackout;
L.bo = bo;

for k = 1:n
    t = tt(k);
    [r, th, v] = target_motion_generator(t, scenario);
    [V, f, d] = turret_controller(t, r, th, v, x(1), x(2));
    s = target_truth(t, scenario); ptrue = s([1 4])';

    % --- evaluator: pulses and hits -------------------------------------
    if f == 1 && last_cmd == 0
        if t - last_issued >= p.t_recharge
            last_issued = t;
            w = [cos(x(1)); sin(x(1))]; u = [-sin(x(1)); cos(x(1))];
            if w'*ptrue >= 0, miss = abs(u'*ptrue); else, miss = norm(ptrue); end
            L.pulses(end+1) = struct('t', t, 'theta', x(1), 'miss', miss, 'hit', miss <= p.R_eng, 'pos', ptrue');
        else
            L.ignored = L.ignored + 1;
        end
    end
    last_cmd = f;

    % --- log -------------------------------------------------------------
    L.theta(k) = x(1); L.thetadot(k) = x(2); L.i(k) = x(3); L.V(k) = V; L.sat(k) = d.ctrl.sat;
    L.ref(k) = d.ref; L.refdot(k) = d.refdot; L.xt(k,:) = ptrue';
    L.mode(k) = find(strcmp(d.aim.mode, {'WAIT', 'TRACK', 'HOLD'}));
    L.valid(k) = v ~= 0; L.blackout(k) = t >= bo(1) && t < bo(2);
    if v, L.meas(k,:) = r*[cos(th) sin(th)]; end
    e = d.est;
    if e.valid
        L.theta_now(k) = e.theta_now; L.theta_pred(k) = e.theta_ref;
        L.xe(k,:) = e.x_now([1 4])'; L.xp(k,:) = e.x_pred([1 4])';
        Pp = e.P_now([1 4],[1 4]); L.Pe(k,:) = [Pp(1,1) Pp(1,2) Pp(2,2)];
        L.sig_now(k) = e.sigma_perp_now; L.sig_pred(k) = e.sigma_perp_pred;
        L.fire_ok(k) = d.fire.ok;
    end
    L.k = k;

    if ~isempty(on_step) && (mod(k-1, every) == 0 || k == n || f == 1)
        if on_step(L, k), break; end
    end
    if k < n, x = plant_rk4_step(t, x, V, dt); end
end
L.metrics = metrics(L, p);
end

% ------------------------------------------------------------------------
function M = metrics(L, p)
P = L.pulses;
M.n_pulses = numel(P);
M.n_hits   = sum([P.hit]);
M.success  = M.n_hits > 0;
if M.success
    M.t_first_hit = P(find([P.hit], 1)).t;
else
    M.t_first_hit = NaN;
end
if M.n_pulses > 0, M.miss = [P.miss]; else, M.miss = []; end
M.ignored = L.ignored;
% turret error against its reference just before each pulse (tiny in a hold;
% the aim switches back to TRACK on the pulse step itself)
if M.n_pulses > 0
    kp = max(round([P.t]/(L.t(2) - L.t(1))), 1);
    M.err_at_pulse = abs(L.ref(kp) - L.theta(kp))';
    M.mode_at_pulse = L.mode(kp)';
else
    M.err_at_pulse = []; M.mode_at_pulse = [];
end
k = 1:L.k;
m = k(L.t(k) > 1 & isfinite(L.ref(k)));
% turret error against its reference while holding (the shooting phase)
h = m(L.mode(m) == 3);
e = L.ref(h) - L.theta(h);
if isempty(e), e = NaN; end
M.hold_err_rms = sqrt(mean(e.^2)); M.hold_err_peak = max(abs(e));
M.hold_frac = numel(h)/numel(m);
M.V_peak = max(abs(L.V(k))); M.sat_frac = mean(L.sat(k));
% across-LOS error of the CURRENT estimate (what the fire decision uses)
u = [-sin(L.theta_now(m)) cos(L.theta_now(m))];
M.est_now_rms = sqrt(mean(sum(u .* (L.xe(m,:) - L.xt(m,:)), 2).^2));
end

function v = getopt(o, name, def)
if isfield(o, name) && ~isempty(o.(name)), v = o.(name); else, v = def; end
end
