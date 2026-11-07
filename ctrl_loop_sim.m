function R = ctrl_loop_sim(ref_fun, refdot_fun, T, opts, x0)
% CTRL_LOOP_SIM  Closed loop of turret_compensator + turret_plant on a
% synthetic reference (no estimator). Used by controller_tests.m.
%   R = ctrl_loop_sim(ref_fun, refdot_fun, T, opts, x0)
%   ref_fun, refdot_fun : function handles of t;  T : duration [s]
%   opts : struct with use_ff, use_aw;  x0 : [theta; thetadot; i] (default 0)
if nargin < 5, x0 = zeros(3,1); end
h  = 1e-3;                       % plant integration step (controller runs at its own 2 ms clock)
tt = (0:h:T)';
n  = numel(tt);
X  = zeros(n, 3); V = zeros(n, 1); ref = zeros(n, 1);
turret_compensator('config', opts);
x = x0(:);
for k = 1:n
    t = tt(k);
    ref(k) = ref_fun(t);
    V(k)   = turret_compensator(t, ref(k), refdot_fun(t), x(1));
    X(k,:) = x';
    if k < n, x = plant_rk4_step(t, x, V(k), h); end
end
turret_compensator('config', struct('use_ff', true, 'use_aw', true));
R.t = tt; R.theta = X(:,1); R.thetadot = X(:,2); R.i = X(:,3); R.V = V; R.ref = ref;
R.e = ref - X(:,1);
end
