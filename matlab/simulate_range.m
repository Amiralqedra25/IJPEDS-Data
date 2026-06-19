function sim = simulate_range(p)
% SIMULATE_RANGE  Longitudinal-dynamics range simulation over the WMTC cycle.
%
%   sim = SIMULATE_RANGE(p) simulates a retrofitted electric motorcycle over
%   the WMTC drive cycle and returns simulated energy, range, and sizing
%   indicators. This is the dynamic counterpart to the framework's
%   steady-state conservative estimate (Range_LB).
%
%   INPUT struct p with fields:
%     p.total_mass_kg : vehicle + rider mass (kg)
%     p.CdA           : effective drag area (m^2)
%     p.Crr           : rolling-resistance coefficient (-)
%     p.rho           : air density (kg/m^3)
%     p.eta_drv       : drivetrain efficiency, traction (-)
%     p.usable_kwh    : usable pack energy (kWh)
%     p.eta_regen     : (optional) regen capture efficiency (default 0.35)
%     p.grade_deg     : (optional) road grade in degrees (default 0, flat road)
%     p.wheel_radius_m: (optional) drive-wheel radius (default 0.28 m)
%
%   OUTPUT struct sim with fields:
%     sim.energy_Wh        : net energy consumed over one cycle (Wh)
%     sim.distance_km      : distance covered in one cycle (km)
%     sim.e_sim_Wh_per_km  : simulated energy intensity (Wh/km)
%     sim.range_sim_km     : simulated range = usable_kWh / e_sim (km)
%     sim.P_peak_kW        : peak wheel power demand over the cycle (kW)
%     sim.T_wheel_peak_Nm  : peak wheel torque demand over the cycle (Nm)
%     sim.P_motor_req_kW   : required motor rating estimate (kW)
%     sim.t, sim.v_kmh     : the cycle used
%     sim.P_wheel_W        : instantaneous wheel power (W)
%     sim.P_batt_W         : instantaneous battery-side power (W)
%
%   MODEL (longitudinal road-load + inertia, 1 Hz):
%     a(k)     = dv/dt
%     F_roll   = m * g * Crr
%     F_aero   = 0.5 * rho * CdA * v^2
%     F_inert  = m * a
%     F_grade  = m * g * sin(theta)     (theta = 0 for flat WMTC)
%     F_trac   = F_roll + F_aero + F_inert + F_grade
%     P_wheel  = F_trac * v
%     T_wheel  = F_trac * R_wheel
%   Traction energy is divided by eta_drv; braking (negative) energy is
%   partially recovered at eta_regen. The grade term is included for
%   completeness and future hilly-terrain analysis; with theta = 0 it does
%   not affect the flat-cycle results reported in the paper.
%
%   Author: RaaS thesis project, IIUM. Deterministic / reproducible.

    if ~isfield(p, 'eta_regen');      p.eta_regen = 0.35;       end
    if ~isfield(p, 'grade_deg');      p.grade_deg = 0;          end
    if ~isfield(p, 'wheel_radius_m'); p.wheel_radius_m = 0.28;  end
    g = 9.81;

    [t, v_kmh] = wmtc_cycle();
    v = v_kmh / 3.6;                 % m/s
    dt = [1; diff(t)];               % s (1 Hz)

    % Acceleration (forward difference)
    a = [diff(v); 0] ./ dt;          % m/s^2

    % Road-load forces
    theta = deg2rad(p.grade_deg);
    F_roll  = p.total_mass_kg * g * p.Crr * (v > 0.1);   % only when moving
    F_aero  = 0.5 * p.rho * p.CdA .* v.^2;
    F_inert = p.total_mass_kg * a;
    F_grade = p.total_mass_kg * g * sin(theta) * (v > 0.1);

    F_trac = F_roll + F_aero + F_inert + F_grade;   % N (negative on braking)
    P_wheel = F_trac .* v;                          % W (negative on braking)
    T_wheel = F_trac * p.wheel_radius_m;            % Nm at the wheel

    % Battery-side power: traction scaled by 1/eta_drv, braking recovered
    % at eta_regen.
    P_batt = zeros(size(P_wheel));
    pos = P_wheel >= 0;
    P_batt(pos)  = P_wheel(pos) / p.eta_drv;
    P_batt(~pos) = P_wheel(~pos) * p.eta_regen;

    % Integrate energy (Wh) and distance (km)
    energy_Wh   = sum(P_batt .* dt) / 3600;
    distance_km = sum(v .* dt) / 1000;

    e_sim     = energy_Wh / distance_km;          % Wh/km
    range_sim = p.usable_kwh * 1000 / e_sim;      % km

    % Sizing indicators (positive traction demand)
    P_peak_W       = max(P_wheel);                % peak wheel power (W)
    T_wheel_peak   = max(T_wheel);                % peak wheel torque (Nm)
    P_motor_req_kW = (P_peak_W / p.eta_drv) / 1000;  % required motor rating

    sim = struct();
    sim.energy_Wh       = energy_Wh;
    sim.distance_km     = distance_km;
    sim.e_sim_Wh_per_km = e_sim;
    sim.range_sim_km    = range_sim;
    sim.P_peak_kW       = P_peak_W / 1000;
    sim.T_wheel_peak_Nm = T_wheel_peak;
    sim.P_motor_req_kW  = P_motor_req_kW;
    sim.t               = t;
    sim.v_kmh           = v_kmh;
    sim.P_wheel_W       = P_wheel;
    sim.P_batt_W        = P_batt;
end
