function [V_m, fire_cmd, dbg] = turret_controller(t, r_meas, theta_meas, meas_valid, theta_turret, theta_dot_turret)
% TURRET_CONTROLLER  Complete predictive-feedback controller (signature fixed
% by the assignment; the third output is optional, for logging only).
%   [V_m, fire_cmd] = turret_controller(t, r_meas, theta_meas, meas_valid, theta_turret, theta_dot_turret)
%   turret_controller('reset')   clears every module's memory
%
% Information flow (assignment Fig. 2), one call per simulation step:
%   radar -> target_estimator   Singer Kalman filter, 0.5 s prediction, theta_ref
%         -> fire_check         pulse if the ACTUAL beam passes the current estimate
%                               within R_eng with margin (uncertainty gating)
%         -> aim_planner        lead-and-wait: reference = predicted angle, held
%         -> turret_compensator Gc(s) at 500 Hz + velocity feed-forward,
%                               +/-300 V with anti-windup -> V_m
% theta_dot_turret is used by fire_check (how fast the beam sweeps) and by
% aim_planner (is the turret really locked on); the feedback loop uses angle only.
%
% Time handling: a call with t = 0 after a run, or a jump back by more than
% 0.1 s, starts a new run and resets everything. Smaller backward steps (an
% adaptive ODE solver retrying a step) are treated as "same time" so they
% cannot corrupt the filters' memories.

persistent last_t
if ischar(t)
    reset_all(); last_t = []; V_m = []; fire_cmd = []; dbg = []; return
end
if isempty(last_t) || (t <= 1e-12 && last_t > 1e-12) || t < last_t - 0.1
    reset_all(); last_t = -Inf;
end
te = max(t, last_t);                 % monotonic time for the modules
last_t = te;

est = target_estimator(te, r_meas, theta_meas, meas_valid);
[fire_cmd, finfo] = fire_check(te, est, theta_turret, theta_dot_turret);
[ref, refdot, aim] = aim_planner(te, est, theta_turret, fire_cmd, theta_dot_turret);
[V_m, cinfo] = turret_compensator(te, ref, refdot, theta_turret);

if nargout > 2
    dbg = struct('est', est, 'ref', ref, 'refdot', refdot, 'aim', aim, ...
                 'fire', finfo, 'ctrl', cinfo);
end
end

function reset_all()
target_estimator('reset'); fire_check('reset'); aim_planner('reset'); turret_compensator('reset');
end
