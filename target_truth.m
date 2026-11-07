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
    if strcmp(spec.kind, 'random')
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
