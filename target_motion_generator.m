function [r_meas, theta_meas, meas_valid] = target_motion_generator(t, scenario)
% TARGET_MOTION_GENERATOR  Radar measurements at simulation time t.
%   [r_meas, theta_meas, meas_valid] = target_motion_generator(t, scenario)
%   meas_valid = 1 only on the FIRST call after a new 20 Hz radar sample;
%   otherwise r_meas = theta_meas = NaN and meas_valid = 0.
%   During a blackout no samples are delivered.
%
% TRICK 1 (timing): the simulation calls this every sim step (e.g. 1 ms),
% but the radar only samples every 50 ms. Sample k is taken at t_k = k*dt;
% it is handed out once, on the first call with t >= t_k.
% TRICK 1b (repeatable noise): noise for sample k is drawn once from a seeded
% table, so re-running or calling twice gives the same measurement.

persistent G
p    = est_params();
spec = scenario_spec(scenario);
key  = sprintf('%s|%d', spec.traj_id, spec.noise_seed);

if isempty(G) || ~strcmp(G.key, key) || t < G.last_t
    G.key    = key;
    G.last_k = -1;
    K = ceil(10/p.dt_radar) + 2;
    st = rng; rng(7919*spec.noise_seed + 13);
    G.e_r  = randn(K, 1);
    G.e_th = randn(K, 1);
    rng(st);
end
G.last_t = t;

r_meas = NaN; theta_meas = NaN; meas_valid = 0;
k = floor(t/p.dt_radar + 1e-9);
if k > G.last_k
    G.last_k = k;
    tk = k*p.dt_radar;
    [s, info] = target_truth(tk, scenario);
    bo = info.blackout;
    if ~(tk >= bo(1) && tk < bo(2))
        r_true  = hypot(s(1), s(4));
        th_true = atan2(s(4), s(1));
        idx = min(k+1, numel(G.e_r));
        r_meas     = r_true  + p.sigma_r  * G.e_r(idx);
        theta_meas = th_true + p.sigma_th * G.e_th(idx);
        meas_valid = 1;
    end
end
end
