function xp = plant_rk4_step(t, x, V, h)
% PLANT_RK4_STEP  One 4th-order Runge-Kutta step of turret_plant with the
% motor voltage V held constant over the step (zero-order hold).
%   x = [theta; thetadot; i]
f  = @(tt, s) plant_f(tt, s, V);
k1 = f(t,       x);
k2 = f(t + h/2, x + h/2*k1);
k3 = f(t + h/2, x + h/2*k2);
k4 = f(t + h,   x + h*k3);
xp = x + h/6*(k1 + 2*k2 + 2*k3 + k4);
end

function d = plant_f(t, s, V)
[d1, d2, d3] = turret_plant(t, V, s(1), s(2), s(3));
d = [d1; d2; d3];
end
