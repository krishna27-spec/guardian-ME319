function c = ctrl_params()
% CTRL_PARAMS  Turret plant constants (assignment Section 2) and the frozen
% controller design of the controls section (Appendix B). Change values ONLY
% here; nothing is retuned per scenario.
persistent CACHE
if ~isempty(CACHE), c = CACHE; return; end   % built once per session (speed)


% --- plant, given by the assignment -------------------------------------
c.J_turret = 1200;      % turret + cradle inertia about the trunnion [kg m^2]
c.J_m      = 0.20;      % motor rotor inertia [kg m^2]
c.N        = 100;       % gear ratio (motor turns N times the turret)
c.b        = 1500;      % turret-side viscous damping [N m s/rad]
c.Kt       = 10;        % torque constant [N m/A]
c.Ke       = 10;        % back-emf constant [V s/rad]
c.Rm       = 1.0;       % armature resistance [ohm]
c.Lm       = 0.10;      % armature inductance [H]
c.theta_lim = [-10 85]*pi/180;   % mechanical stops [rad]
c.J_eq     = c.J_turret + c.N^2*c.J_m;   % 3200 kg m^2, reflected to the turret

% hard stops in turret_plant: stiff one-sided spring-damper beyond the limits
% (never reached inside the design envelope; a safety net only)
c.stop_wn   = 200;      % [rad/s] stop stiffness as a natural frequency
c.stop_zeta = 0.7;

% --- controller, frozen design (controls section, Appendix B) -------------
c.Kc     = 16744;       % compensator gain [V/rad]
c.z_i    = 7.692;       % PI zero [1/s]
c.wn     = 55.94;       % notch frequency = plant resonance [rad/s]
c.zeta_z = 0.0936;      % notch zero damping = plant damping
c.zeta_p = 0.4;         % notch pole damping
c.Kv     = (c.Rm*c.b + c.N^2*c.Kt*c.Ke)/(c.N*c.Kt);   % velocity feed-forward, 1001.5 V s/rad
c.Ts     = 2e-3;        % controller sample time [s] (500 Hz, Tustin)
c.V_max  = 300;         % motor voltage limit [V]
CACHE = c;
end
