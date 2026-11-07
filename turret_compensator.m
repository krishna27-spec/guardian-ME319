function [V, info] = turret_compensator(t, ref, refdot, theta)
% TURRET_COMPENSATOR  Sampled feedback controller of the controls section.
%   [V, info] = turret_compensator(t, ref, refdot, theta)
%   turret_compensator('reset')
%   turret_compensator('config', opts)   opts.use_ff, opts.use_aw (tests only)
%   K = turret_compensator('matrices')    discrete matrices, for the design check
%
%   Gc(s) = Kc (s + z_i)/s * (s^2 + 2 zeta_z wn s + wn^2)/(s^2 + 2 zeta_p wn s + wn^2)
%   (type-2 loop with the plant's integrator; notch zeros on the plant resonance)
%   V = Gc * (ref - theta) + Kv * refdot      (velocity feed-forward, outside the loop)
%
% Implementation (controls section, Sec. 5):
%   * Tustin discretisation at Ts = 2 ms, in state-space form
%   * own 500 Hz clock: the output is updated at t_k = k*Ts and HELD in between,
%     so the controller is correct whatever step the simulation uses (if a call
%     skips several samples, the missed updates are run with the current error)
%   * +/-300 V saturation with Hanus anti-windup on the TOTAL command:
%       x(k+1) = Ad x + Bd e + Bd/Dd (u_sat - u),   u = Cd x + Dd e + Vff

persistent K X OPT
if ischar(t)
    switch t
        case 'reset',    X = [];
        case 'config',   OPT = ref; X = [];
        case 'matrices', if isempty(K), K = comp_matrices(); end
                         V = K; info = []; return
    end
    V = []; info = []; return
end
if isempty(OPT), OPT = struct('use_ff', true, 'use_aw', true); end
if isempty(K), K = comp_matrices(); end
c = K.c;

if isempty(X) || t < X.last_t - 1e-12          % new run
    X = struct('x', zeros(3,1), 'k', 0, 'V', 0, 'u', 0, 'e', 0, 'last_t', -Inf);
end
X.last_t = t;

if t - X.k*c.Ts > 20*c.Ts                       % far behind (late first call): resynchronise
    X.k = floor(t/c.Ts + 1e-9);
end
while t >= X.k*c.Ts - 1e-9                      % run every sample instant that has passed
    e  = ref - theta;
    u  = K.Cd*X.x + K.Dd*e + OPT.use_ff*c.Kv*refdot;
    us = min(max(u, -c.V_max), c.V_max);
    if OPT.use_aw
        X.x = K.Ad*X.x + K.Bd*e + K.Bd/K.Dd*(us - u);
    else
        X.x = K.Ad*X.x + K.Bd*e;
    end
    X.V = us; X.u = u; X.e = e;
    X.k = X.k + 1;
end
V = X.V;
info = struct('e', X.e, 'u', X.u, 'sat', abs(X.u) > c.V_max);
end

% ------------------------------------------------------------------------
function K = comp_matrices()
c = ctrl_params();
% continuous transfer function, numerator and denominator (descending powers)
num = c.Kc * conv([1 c.z_i], [1 2*c.zeta_z*c.wn c.wn^2]);
den = conv([1 0], [1 2*c.zeta_p*c.wn c.wn^2]);
% controllable canonical state space (num and den both degree 3, den monic)
a = den(2:4); b = num;
A = [0 1 0; 0 0 1; -a(3) -a(2) -a(1)];
B = [0; 0; 1];
D = b(1);
C = [b(4) - D*a(3), b(3) - D*a(2), b(2) - D*a(1)];
% Tustin (bilinear) discretisation, s = (2/Ts)(z-1)/(z+1)
K = tustin_ss(A, B, C, D, c.Ts);
K.A = A; K.B = B; K.C = C; K.D = D; K.num = num; K.den = den; K.c = c;
end
