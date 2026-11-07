function [theta_dot_turret, theta_ddot_turret, i_dot] = turret_plant(t, V_m, theta_turret, theta_dot_turret, i)
% TURRET_PLANT  Elevation-drive dynamics (signature fixed by the assignment).
%   [theta_dot_turret, theta_ddot_turret, i_dot] = turret_plant(t, V_m, theta_turret, theta_dot_turret, i)
%   Returns the state derivatives; the simulation integrates them.
%
% First-principles model (controls section, Sec. 3):
%   armature:  V = Rm*i + Lm*di/dt + N*Ke*thetadot      (motor turns N*thetadot)
%   mechanics: J_eq*thetaddot + b*thetadot = N*Kt*i + tau_stop,
%              J_eq = J_turret + N^2*J_m = 3200 kg m^2
% tau_stop is a stiff one-sided spring-damper that acts only beyond the
% mechanical limits -10 deg / +85 deg (it can push, never pull).

c = ctrl_params();

tau_stop = 0;
Ks = c.J_eq*c.stop_wn^2;
Cs = 2*c.stop_zeta*c.stop_wn*c.J_eq;
if theta_turret > c.theta_lim(2)
    tau_stop = min(0, -Ks*(theta_turret - c.theta_lim(2)) - Cs*theta_dot_turret);
elseif theta_turret < c.theta_lim(1)
    tau_stop = max(0, -Ks*(theta_turret - c.theta_lim(1)) - Cs*theta_dot_turret);
end

theta_ddot_turret = (c.N*c.Kt*i - c.b*theta_dot_turret + tau_stop) / c.J_eq;
i_dot = (V_m - c.Rm*i - c.N*c.Ke*theta_dot_turret) / c.Lm;
end
