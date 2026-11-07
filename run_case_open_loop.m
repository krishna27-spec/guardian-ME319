function R = run_case_open_loop(scenario, p, dts)
% RUN_CASE_OPEN_LOOP  Run generator + estimator for one case (no turret) and
% record the prediction error across the line of sight and the NEES.
target_estimator('config', p); clear target_motion_generator
tt = (0:dts:10)';
n = numel(tt);
R.t = tt; R.e_pred = NaN(n,1); R.s_pred = NaN(n,1); R.nees_pos = NaN(n,1);
for i = 1:n
    t = tt(i);
    [r, th, v] = target_motion_generator(t, scenario);
    e = target_estimator(t, r, th, v);
    if ~e.valid, continue; end
    if t + p.T_pred <= tt(end)          % truth only exists up to t_end = 10 s
    sp = target_truth(t + p.T_pred, scenario);
    u = [-sin(e.theta_ref); cos(e.theta_ref)];
    R.e_pred(i) = u' * (e.x_pred([1 4]) - sp([1 4])');
    R.s_pred(i) = e.sigma_perp_pred;
    end
    [sn, info] = target_truth(t, scenario);
    d = e.x_now([1 4]) - sn([1 4])';
    R.nees_pos(i) = d' / e.P_now([1 4],[1 4]) * d;
end
R.blackout = info.blackout;
target_estimator('config', est_params());
end
