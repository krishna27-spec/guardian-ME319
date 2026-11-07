function est = target_estimator(t, r_meas, theta_meas, meas_valid)
% TARGET_ESTIMATOR  Singer-model Kalman filter + 0.5 s predictor.
%   est = target_estimator(t, r_meas, theta_meas, meas_valid)
%   Call once per simulation step. Keeps its memory (x_hat, P) between calls.
%   target_estimator('reset')            clears the memory
%   target_estimator('config', params)   use a modified est_params() struct
%
% State: s = [x vx ax  y vy ay]'  (Cartesian, Singer per axis, W1-W3)
% Measurement: z = [r cos(theta); r sin(theta)],  R = J diag(sr^2, sth^2) J'
%
% Output fields (all angles in rad):
%   valid          false until the first radar dot arrived
%   theta_ref      predicted elevation at t + T_pred      -> controller reference
%   thetadot_ref   its rate (for feed-forward)
%   theta_ref_sat  theta_ref clipped to [-9.7, 84.7] deg (p.theta_ref_lim)
%   thetadot_ref_sat  thetadot_ref, set to 0 while theta_ref is clipped
%   n_rep          number of repeated (ignored) dots so far (TRICK 8)
%   x_now, P_now   estimate propagated to the current time t (for fire logic)
%   x_pred,P_pred  estimate at t + T_pred
%   sigma_perp_now/pred  std across the line of sight [m]
%   age            time since the last radar dot [s]
%   nis            normalised innovation squared of the last update (chi2, 2 dof)

persistent S CFG
if ischar(t)
    switch t
        case 'reset',  S = [];
        case 'config', CFG = r_meas; S = [];
    end
    est = [];
    return
end
if isempty(CFG), CFG = est_params(); end
p = CFG;

if isempty(S) || t < S.last_t - 1e-12      % new run detected
    S = struct('init', false, 'x', zeros(6,1), 'P', eye(6), 't_est', 0, ...
               't_first', NaN, 't_meas', NaN, 'last_t', -Inf, 'nis', NaN, ...
               'last_r', NaN, 'last_th', NaN, 'n_rep', 0);
end
S.last_t = t;

H = zeros(2, 6); H(1,1) = 1; H(2,4) = 1;
have = (meas_valid ~= 0) && isfinite(r_meas) && isfinite(theta_meas);
% TRICK 8 (repeated dots): if a generator holds the last reading with
% meas_valid = 1 between radar samples, the same dot would be fused many
% times and P would collapse. Genuine dots carry fresh noise, so an exact
% repeat of BOTH numbers is never a new measurement. (NaN == x is false,
% so the very first dot always counts as new.)
if have && r_meas == S.last_r && theta_meas == S.last_th
    have = false;
    S.n_rep = S.n_rep + 1;
end

if have
    z = [r_meas*cos(theta_meas); r_meas*sin(theta_meas)];
    R = meas_cov(r_meas, theta_meas, p);
    if ~S.init
        % TRICK 3 (initialisation): position from the first dot, velocity and
        % acceleration unknown -> zero mean with a deliberately huge variance,
        % so the next few dots dominate. Firing waits p.settle_time.
        S.x = [z(1); 0; 0; z(2); 0; 0];
        S.P = zeros(6);
        S.P([1 4],[1 4]) = R;
        S.P(2,2) = p.v_max^2;   S.P(5,5) = p.v_max^2;
        S.P(3,3) = p.sigma_m^2; S.P(6,6) = p.sigma_m^2;
        S.init = true; S.t_first = t;
    else
        % predict from the time of the last estimate to now ...
        [S.x, S.P] = predict(S.x, S.P, t - S.t_est, p);
        % ... then update (Ch. 3: x = A + K (B - A))
        nu = z - H*S.x;
        Sinn = H*S.P*H' + R;
        K  = S.P*H' / Sinn;
        S.x = S.x + K*nu;
        IKH = eye(6) - K*H;
        % TRICK 2: Joseph form keeps P symmetric positive definite
        S.P = IKH*S.P*IKH' + K*R*K';
        S.P = (S.P + S.P')/2;
        S.nis = nu' / Sinn * nu;
    end
    S.t_est = t; S.t_meas = t;
    S.last_r = r_meas; S.last_th = theta_meas;
end

est = empty_output();
est.t = t;
if ~S.init
    return                                   % nothing known yet
end
est.valid = true;
est.age = t - S.t_meas;
est.t_since_init = t - S.t_first;
est.nis = S.nis;

% TRICK 4: the filter's own memory is NOT moved forward between dots.
% Outputs are predictions from the last estimate, so they stay smooth and
% a blackout (no dots) simply means predicting further: P grows by itself.
[est.x_now,  est.P_now ] = predict(S.x, S.P, t - S.t_est, p);
[est.x_pred, est.P_pred] = predict(S.x, S.P, t + p.T_pred - S.t_est, p);

[est.theta_now, est.r_now, ~, est.sigma_perp_now] = los(est.x_now, est.P_now);
[est.theta_ref, est.r_pred, est.thetadot_ref, est.sigma_perp_pred] = los(est.x_pred, est.P_pred);
% reference handed to the controller: clamped 0.3 deg inside the stops, and
% its rate set to zero while clamped (requested by the controls section)
est.theta_ref_sat = min(max(est.theta_ref, p.theta_ref_lim(1)), p.theta_ref_lim(2));
est.thetadot_ref_sat = est.thetadot_ref;
if est.theta_ref_sat ~= est.theta_ref, est.thetadot_ref_sat = 0; end
est.n_rep = S.n_rep;
end

% ------------------------------------------------------------------------
function [x, P] = predict(x, P, dt, p)
if dt <= 0, return; end
[F3, Q3] = singer_cached(dt, p);
F = zeros(6); F(1:3,1:3) = F3; F(4:6,4:6) = F3;   % x and y axes independent
Q = zeros(6); Q(1:3,1:3) = Q3; Q(4:6,4:6) = Q3;   % (same as blkdiag, faster)
x = F*x;
P = F*P*F' + Q;
P = (P + P')/2;
end

function [F3, Q3] = singer_cached(dt, p)
% Speed only: with a fixed sim step the same few dt values repeat every
% radar cycle, so expm is computed once per distinct dt (rounded to 1 us).
persistent KEYS VALS TAG
tag = [p.tau_m p.sigma_m];
if isempty(KEYS) || ~isequal(TAG, tag)
    KEYS = zeros(0, 1); VALS = {}; TAG = tag;
end
key = round(dt*1e6);
j = find(KEYS == key, 1);
if ~isempty(j)
    F3 = VALS{j, 1}; Q3 = VALS{j, 2};
else
    [F3, Q3] = singer_FQ(dt, p.tau_m, p.sigma_m);
    if numel(KEYS) < 5000, KEYS(end+1, 1) = key; VALS(end+1, :) = {F3, Q3}; end
end
end

function R = meas_cov(r, th, p)
% R_xy = J R_rtheta J'  (your derivation; the ellipse turns with the beam)
c = cos(th); s = sin(th);
J = [c, -r*s; s, r*c];
R = J * diag([p.sigma_r^2, p.sigma_th^2]) * J';
end

function [th, r, thdot, sig_perp] = los(x, P)
px = x(1); py = x(4); vx = x(2); vy = x(5);
r  = hypot(px, py);
th = atan2(py, px);
thdot = (px*vy - py*vx) / r^2;
u = [-sin(th); cos(th)];                     % unit vector across the line of sight
sig_perp = sqrt(u' * P([1 4],[1 4]) * u);
end

function est = empty_output()
est = struct('valid', false, 't', NaN, 'age', NaN, 't_since_init', NaN, 'nis', NaN, ...
    'x_now', NaN(6,1), 'P_now', NaN(6), 'x_pred', NaN(6,1), 'P_pred', NaN(6), ...
    'theta_now', NaN, 'r_now', NaN, 'sigma_perp_now', NaN, ...
    'theta_ref', NaN, 'theta_ref_sat', NaN, 'r_pred', NaN, 'thetadot_ref', NaN, ...
    'thetadot_ref_sat', NaN, 'sigma_perp_pred', NaN, 'n_rep', 0);
end
