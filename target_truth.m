function [s, info] = target_truth(t, scenario)
% TARGET_TRUTH  True target state at time(s) t.
%   s = [x vx ax y vy ay] (one row per entry of t)
%   info.blackout = [onset end), info.spec
% The trajectory is built once on a 1 ms grid and cached, then interpolated.

persistent cache
spec = scenario_spec(scenario);
if isempty(cache) || ~strcmp(cache.id, spec.traj_id)
    cache.id  = spec.traj_id;
    cache.tab = build_trajectory(spec);
    cache.blackout = spec.blackout;
    if any(strcmp(spec.kind, {'random', 'wild'}))
        cache.blackout = cache.tab.blackout;
    end
end
tab = cache.tab;
% uniform 1 ms grid -> direct linear interpolation (much faster than interp1)
t  = min(max(t(:), 0), tab.t(end));
h  = tab.t(2) - tab.t(1);
i0 = min(floor(t/h) + 1, numel(tab.t) - 1);
w  = t/h - (i0 - 1);
s  = tab.S(i0,:) .* (1 - w) + tab.S(i0+1,:) .* w;
info.blackout = cache.blackout;
info.spec = spec;
end

% ------------------------------------------------------------------------
function tab = build_trajectory(spec)
p  = est_params();
tt = (0:1e-3:10)';

switch spec.kind
    case 'line'
        tab = integrate(tt, [1500 900], [-100 -20], zeros(numel(tt), 2));
    case 'manoeuvre'
        acc = manoeuvre(tt, 1.0, [12 12], [0.8 1.1], [0 pi/2]);
        tab = integrate(tt, [1500 900], [-100 -20], acc);
    case 'random'
        st = rng_push(1000 + spec.seed);
        ok = false;
        for attempt = 1:500
            r0  = 1300 + 600*rand;            el0 = (5 + 55*rand)*pi/180;
            spd = 60 + 80*rand;               hdg = 2*pi*rand;
            amp = rand(1,2); amp = (8 + 11.5*rand) * amp/norm(amp);   % |a| <= 19.5
            w   = 0.3 + 1.2*rand(1,2);        phi = 2*pi*rand(1,2);
            t_on = 1 + 2*rand;
            acc = manoeuvre(tt, t_on, amp, w, phi);
            tab = integrate(tt, r0*[cos(el0) sin(el0)], spd*[cos(hdg) sin(hdg)], acc);
            if limits_ok(tab, p), ok = true; break; end
        end
        bo_on = 1 + 8.5*rand;  bo_len = 0.05 + 0.45*rand;
        tab.blackout = [bo_on, bo_on + bo_len];
        rng_pop(st);
        if ~ok, error('could not draw an admissible random case %d', spec.seed); end
        return
    case 'wild'
        st = rng_push(5000 + spec.seed);
        ok = false;
        for attempt = 1:300
            r0  = 1250 + 400*rand;            el0 = (15 + 50*rand)*pi/180;
            spd = 25 + 45*rand;               hdg = 2*pi*rand;   % slow = tighter turns
            p0  = r0*[cos(el0) sin(el0)];     v0  = spd*[cos(hdg) sin(hdg)];
            acc = wild_accel(tt, p0, v0, 0.5 + 0.5*rand, p);
            tab = integrate(tt, p0, v0, acc);
            if limits_ok(tab, p), ok = true; break; end
        end
        tab.blackout = random_blackouts();
        rng_pop(st);
        if ~ok, error('could not draw an admissible wild case W%d', spec.seed); end
        return
end
% TRICK 7: the generator must enforce the limits, so every fixed scenario is checked
if ~limits_ok(tab, p)
    error('scenario %s violates the range/speed/acceleration limits', spec.name);
end
end

function acc = manoeuvre(tt, t_on, amp, w, phi)
% smooth, bounded: sinusoids switched on with a C1 smoothstep over 0.5 s,
% so acceleration is continuous (no jerk spikes) and |a| <= norm(amp)
tau  = max(tt - t_on, 0);
ramp = min(tau/0.5, 1); ramp = 3*ramp.^2 - 2*ramp.^3;
acc  = [amp(1)*sin(w(1)*tau + phi(1)), amp(2)*sin(w(2)*tau + phi(2))] .* ramp;
end

function acc = wild_accel(tt, p0, v0, t_on, p)
% WILD manoeuvre: the drone keeps changing its mind. At random instants
% (every 0.3-1.5 s) it picks a new command in its own velocity frame:
% a hard turn left or right (lateral, 85-100 % of 19.5 m/s^2) plus a random
% speed-up or slow-down (tangential, up to 80 % of it). Slow drones
% (25-70 m/s at the start) turn tighter, since the turn rate is a/v. Commands are blended with a smoothstep
% over 0.2 s, so acceleration stays continuous. Soft "walls" steer it back
% before it would leave the range / elevation / speed limits.
A = 19.5; h = tt(2) - tt(1); n = numel(tt);
% command schedule: switch times and [lateral tangential] commands.
% The turn side is kept with probability 0.75, so turns often last 1-4 s
% (long arcs and loops), not just a zigzag that averages out.
ts = t_on; while ts(end) < tt(end), ts(end+1) = ts(end) + 0.3 + 1.2*rand; end
cmd = zeros(numel(ts), 2); side = sign(rand - 0.5);
for j = 1:numel(ts)
    if rand > 0.75, side = -side; end
    an = side * A*(0.85 + 0.15*rand);                 % hard turn
    at = A*(2*rand - 1)*0.8;                          % speed up / slow down
    c = [an at]; if norm(c) > A, c = A*c/norm(c); end
    cmd(j,:) = c;
end
acc = zeros(n, 2); x = p0; v = v0; j = 0; prev = [0 0];
for k = 1:n
    t = tt(k);
    while j < numel(ts) && t >= ts(j+1), j = j + 1; if j > 1, prev = cmd(j-1,:); end, end
    if j == 0, c = [0 0];
    else
        b = min((t - ts(j))/0.2, 1); b = 3*b^2 - 2*b^3;
        c = prev*(1 - b) + cmd(j,:)*b;
    end
    spd = norm(v); ev = v/max(spd, 1e-6); en = [-ev(2) ev(1)];
    at = c(2) + 1.5*max(20 - spd, 0) - 1.5*max(spd - 130, 0);    % speed walls
    a = c(1)*en + at*ev;
    r = norm(x); er = x/r; el = atan2(x(2), x(1)); ee = [-sin(el) cos(el)];
    a = a - 0.08*max(r - 1800, 0)*er + 0.08*max(1000 - r, 0)*er ...         % range walls
          - 0.6*r*max(el - 75*pi/180, 0)*ee + 0.6*r*max(0 - el, 0)*ee;      % elevation walls
    if norm(a) > A, a = A*a/norm(a); end
    if t < t_on, a = [0 0]; end
    acc(k,:) = a;
    v = v + a*h; x = x + v*h;
end
end

function bo = random_blackouts()
% 1-3 blackouts, each: onset >= 1 s, length 0.15-0.5 s, at least 0.1 s apart
nb = 1 + floor(3*rand); bo = [Inf Inf]; t = 1;
for j = 1:nb
    on = t + 2.5*rand; len = 0.15 + 0.35*rand;
    if on + len > 9.9, break; end
    if j == 1, bo = [on on+len]; else, bo(end+1,:) = [on on+len]; end
    t = on + len + 0.1;
end
end

function tab = integrate(tt, p0, v0, acc)
v = v0 + cumtrapz(tt, acc);
x = p0 + cumtrapz(tt, v);
tab.t = tt;
tab.S = [x(:,1) v(:,1) acc(:,1) x(:,2) v(:,2) acc(:,2)];
end

function ok = limits_ok(tab, p)
r   = hypot(tab.S(:,1), tab.S(:,4));
spd = hypot(tab.S(:,2), tab.S(:,5));
acc = hypot(tab.S(:,3), tab.S(:,6));
el  = atan2(tab.S(:,4), tab.S(:,1));
ok  = all(r >= 800) && all(r <= 2000) && all(spd <= p.v_max) && ...
      all(acc <= p.a_max + 1e-9) && all(el >= p.theta_lim(1)) && all(el <= p.theta_lim(2));
end

function st = rng_push(seed)
st = rng; rng(seed);          % works in MATLAB and Octave >= 7
end
function rng_pop(st)
rng(st);
end
