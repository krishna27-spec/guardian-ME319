function [fire_cmd, info] = fire_check(t, est, theta_turret, theta_dot_turret)
% FIRE_CHECK  Decide whether to issue a laser pulse NOW.
%   [fire_cmd, info] = fire_check(t, est, theta_turret, theta_dot_turret)
%   fire_check('reset')
%
% The laser is instantaneous, so the check uses the CURRENT target estimate
% (est.x_now), not the 0.5 s prediction: the beam must pass the target where
% it is now.
%
% TRICK 5 (uncertainty gating): fire only if
%       miss + k * sigma  <=  R_eng
%   miss  = distance from the actual turret ray to the estimated target
%   sigma = estimate std across that ray (from P_now)
% During a blackout P grows, so sigma grows and firing stops by itself;
% right after start-up P is large, so it waits for convergence too.
%
% TRICK 9 (closest approach): inside that window, wait until the beam is
% nearest the target. With d = u'*pos the signed miss, its rate is
%       d_dot = u'*v - theta_dot_turret * (w'*pos)
%   (target velocity across the beam, minus the beam sweeping past it).
% If d*d_dot < 0 they are still closing, so a better shot is coming; fire
% once they stop closing, when the crossing is less than fire_tc away, or
% when the miss is already below fire_good. In a lead-and-wait hold the
% target is fired on as it crosses the beam, not as it enters the window.
%
% TRICK 6 (pulse edges): a pulse needs a 0->1 edge and >= 1 s since the last
% ISSUED pulse. We return 1 for a single call, then 0, and keep our own
% recharge timer with a small margin so the evaluator never ignores an edge.

persistent FS
if nargin < 4, theta_dot_turret = 0; end
p = est_params();
if ischar(t)
    FS = []; fire_cmd = []; info = []; return
end
if isempty(FS) || t < FS.last_t
    FS = struct('last_pulse', -Inf, 'last_cmd', 0, 'last_t', -Inf);
end
FS.last_t = t;

info = struct('miss', NaN, 'sigma', NaN, 'd_dot', NaN, 'ok', false, 'best', false);
fire_cmd = 0;
if est.valid && est.t_since_init >= p.settle_time
    w = [cos(theta_turret); sin(theta_turret)];     % along the ray
    u = [-sin(theta_turret); cos(theta_turret)];    % across the ray
    pos = est.x_now([1 4]); vel = est.x_now([2 5]);
    in_front = (w' * pos) > 0;
    d = u' * pos;
    info.miss  = abs(d);
    info.sigma = sqrt(u' * est.P_now([1 4],[1 4]) * u);
    info.d_dot = u' * vel - theta_dot_turret * (w' * pos);
    info.ok = in_front && (info.miss + p.fire_k*info.sigma <= p.R_eng);
    closing = d * info.d_dot < 0;
    info.best = ~closing || info.miss <= abs(info.d_dot)*p.fire_tc || info.miss <= p.fire_good;
    ready = (t - FS.last_pulse) >= p.t_recharge + p.recharge_eps;
    if info.ok && info.best && ready && FS.last_cmd == 0
        fire_cmd = 1;
        FS.last_pulse = t;
    end
end
FS.last_cmd = fire_cmd;
end
