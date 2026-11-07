function [ref, refdot, A] = aim_planner(t, est, theta_turret, pulse_now, theta_dot_turret)
% AIM_PLANNER  Lead-and-wait aiming: turns the 0.5 s prediction into the
% turret reference.
%   [ref, refdot, A] = aim_planner(t, est, theta_turret, pulse_now, theta_dot_turret)
%   aim_planner('reset')
%
% Why not track theta_ref = theta_hat(t + 0.5) continuously? The laser has no
% time of flight and the controller tracks its reference almost perfectly,
% so the beam would always sit 0.5 s AHEAD of the target (8-31 m) and never
% pass through it. Instead the 0.5 s prediction is used as an intercept point:
%
%   TRACK  ref = theta_hat(t + T_pred), refdot = its rate (the PDF's reference,
%          with feed-forward). The turret runs 0.5 s ahead of the target.
%   HOLD   when the turret is locked on that moving reference (angle within
%          aim_lock_tol AND rate within aim_lock_rate_tol, so a turret that is
%          merely slewing THROUGH the reference does not count), freeze it:
%          ref = theta_hat(t_hold + T_pred), refdot = 0. The target flies into
%          the stationary beam about T_pred later, and fire_check fires as it
%          crosses (judged on the CURRENT estimate, sigma ~ 0.4 m).
%   back to TRACK after a pulse, when the target has passed the beam, or
%          after aim_hold_max without a crossing (manoeuvre, poor prediction).
%
% Hold timing: a hold starts only if the laser will have recharged by the
% time the target arrives (t + T_pred >= last pulse + t_recharge + margin),
% so every hold can end in a pulse. Before the first radar dot the turret is
% told to stay where it is (WAIT).
%
% Output A: mode ('WAIT'/'TRACK'/'HOLD'), hold angle, number of holds,
% reason for the last switch.

persistent S
if nargin < 5, theta_dot_turret = 0; end
if ischar(t)
    S = []; ref = []; refdot = []; A = []; return
end
if isempty(S) || t < S.last_t - 1e-12
    S = struct('mode', 'WAIT', 'hold', NaN, 't_hold', NaN, 'side', 0, ...
               'last_pulse', -Inf, 'n_holds', 0, 'reason', 'start', 'last_t', -Inf);
end
S.last_t = t;
p = est_params();

if pulse_now
    S.last_pulse = t;
    if strcmp(S.mode, 'HOLD'), S.mode = 'TRACK'; S.reason = 'fired'; end
end

if ~est.valid                                   % nothing known yet: stay put
    S.mode = 'WAIT'; ref = theta_turret; refdot = 0;
    A = out(S); return
end
if strcmp(S.mode, 'WAIT'), S.mode = 'TRACK'; S.reason = 'first dot'; end

pos = est.x_now([1 4]);
if strcmp(S.mode, 'HOLD')
    u = [-sin(S.hold); cos(S.hold)];            % across the held beam
    d = u' * pos;                               % signed miss of the current estimate
    passed  = sign(d) ~= S.side && abs(d) > p.R_eng;
    timeout = (t - S.t_hold) > p.aim_hold_max;
    if passed || timeout
        S.mode = 'TRACK';
        if passed, S.reason = 'passed'; else, S.reason = 'timeout'; end
    end
end

if strcmp(S.mode, 'TRACK')
    ref = est.theta_ref_sat; refdot = est.thetadot_ref_sat;
    locked    = abs(theta_turret - ref) <= p.aim_lock_tol && ...
                abs(theta_dot_turret - refdot) <= p.aim_lock_rate_tol;
    confident = est.t_since_init >= p.settle_time;
    ready     = t + p.T_pred >= S.last_pulse + p.t_recharge + p.recharge_eps + p.aim_margin;
    if locked && confident && ready
        S.mode = 'HOLD'; S.hold = ref; S.t_hold = t; S.n_holds = S.n_holds + 1;
        u = [-sin(S.hold); cos(S.hold)];
        S.side = sign(u' * pos);                % which side of the beam the target is on now
        S.reason = 'locked';
    end
end
if strcmp(S.mode, 'HOLD')
    ref = S.hold; refdot = 0;
end
A = out(S);
end

function A = out(S)
A = struct('mode', S.mode, 'hold', S.hold, 't_hold', S.t_hold, ...
           'n_holds', S.n_holds, 'reason', S.reason);
end
