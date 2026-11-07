function p = est_params()
% EST_PARAMS  All estimation, prediction, aiming and fire-logic parameters.
% Every number here is either given by the assignment or justified in the
% study log (Singer model, W5). Change values ONLY here, never per scenario.
persistent CACHE
if ~isempty(CACHE), p = CACHE; return; end   % built once per session (speed)


% --- given by the assignment -------------------------------------------
p.dt_radar   = 0.05;             % radar period [s] (20 Hz)
p.T_pred     = 0.50;             % prediction horizon t_p [s]
p.sigma_r    = 1.0;              % range noise std [m]
p.sigma_th   = 0.02*pi/180;      % angle noise std [rad]
p.a_max      = 20;               % acceleration bound [m/s^2]
p.v_max      = 150;              % speed bound [m/s]
p.R_eng      = 2.0;              % engagement radius [m]
p.t_recharge = 1.0;              % min time between laser pulses [s]
p.theta_lim  = [-10 85]*pi/180;  % turret elevation limits [rad]

% --- our design choices (justify in paper) ------------------------------
p.tau_m   = 2.0;                 % Singer manoeuvre time constant [s] (frozen after the T3 sensitivity test)
p.sigma_m = p.a_max/sqrt(3);     % Singer accel std per axis [m/s^2] (uniform on [-a_max, a_max]; conservative, see paper)

% reference clamp requested by the controls section: stay 0.3 deg inside the
% stops so the turret's overshoot at the clamp (0.19 deg) never reaches them
p.theta_ref_lim = [-9.7 84.7]*pi/180;

% --- fire logic --------------------------------------------------------
p.fire_k       = 2.0;            % confidence multiplier: fire if miss + k*sigma <= R_eng
p.settle_time  = 0.5;            % [s] after first radar dot before firing is allowed
p.recharge_eps = 1e-3;           % [s] safety margin on the recharge timer (TRICK 6)
p.fire_tc      = 2e-3;           % [s] fire if the beam crosses the estimate within this time (TRICK 9)
p.fire_good    = 0.3;            % [m] ...or if the estimated miss is already this small

% --- lead-and-wait aiming (aim_planner) ---------------------------------
p.aim_lock_tol = 1.0e-3;         % [rad] turret counts as locked on the moving reference ...
p.aim_lock_rate_tol = 5e-3;      % [rad/s] ... only if its rate also matches the reference rate
p.aim_margin   = 0.05;           % [s] hold starts so the target arrives this long after the laser is ready
p.aim_hold_max = 1.0;            % [s] give up a hold that has not produced a crossing (= T_pred + 0.5 s)
CACHE = p;
end
