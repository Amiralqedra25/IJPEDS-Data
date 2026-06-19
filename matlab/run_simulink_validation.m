% run_simulink_validation.m
% ==========================
% Runs the Simulink block-diagram model (raas_powertrain_model.slx) for all
% five representative retrofit cases and validates it three ways:
%   1. Simulink simulated range  vs  script-model simulated range
%      (these should agree within ~3%, small numerical differences come from
%       continuous-time ode45 integration vs the 1 Hz discrete script)
%   2. Simulink simulated range  vs  framework Range_LB
%      (Range_LB must sit BELOW the simulated range in every case, because
%       the recalibrated framework is a conservative floor)
%   3. Prints the consolidated validation table and saves it as CSV.
%
% USAGE (in MATLAB, from this folder):
%   >> build_simulink_model      % once, creates the .slx
%   >> run_simulink_validation   % runs all five cases
%
% Requires: Simulink. Files: raas_params.csv, wmtc_cycle.m,
%           simulate_range.m, raas_powertrain_model.slx

clear; clc;

mdl = 'raas_powertrain_model';
if ~exist([mdl '.slx'], 'file')
    error('Model not found. Run build_simulink_model first.');
end
load_system(mdl);

% Drive cycle (same deterministic WMTC profile as the script model)
[t, v_kmh] = wmtc_cycle();
v = v_kmh / 3.6;                       % m/s
a = [diff(v); 0] ./ [1; diff(t)];      % m/s^2 (1 Hz forward difference)

% From Workspace inputs and stop time (base workspace)
assignin('base', 'v_ts', [t v]);
assignin('base', 'a_ts', [t a]);
assignin('base', 't_end', t(end));
assignin('base', 'eta_regen', 0.35);
assignin('base', 'grade_rad', 0);          % flat road (theta = 0) for the paper
assignin('base', 'wheel_radius_m', 0.28);  % typical motorcycle drive-wheel radius

T = readtable('raas_params.csv');
n = height(T);
fprintf('Simulink validation over %d representative cases\n\n', n);

% Preallocate (avoids the table-growth warning)
v_fam   = strings(n,1);
v_donor = strings(n,1);
v_rLB   = zeros(n,1);
v_rScr  = zeros(n,1);
v_rSlx  = zeros(n,1);
v_pct   = zeros(n,1);
v_floor = false(n,1);
v_Ppk   = zeros(n,1);
v_Tpk   = zeros(n,1);
v_Pmot  = zeros(n,1);

for i = 1:n
    % Per-case parameters into base workspace (blocks reference these names)
    assignin('base', 'mass',    T.total_mass_kg(i));
    assignin('base', 'Crr',     T.Crr(i));
    assignin('base', 'CdA',     T.CdA(i));
    assignin('base', 'rho',     T.rho(i));
    assignin('base', 'eta_drv', T.eta_drv(i));

    % Run the Simulink model
    out = sim(mdl, 'ReturnWorkspaceOutputs', 'on');
    E_Wh_v   = out.get('E_Wh');     E_Wh   = E_Wh_v(end);
    dist_v   = out.get('dist_km'); dist_km = dist_v(end);
    e_slx    = E_Wh / dist_km;                       % Wh/km
    rng_slx  = T.usable_kwh(i) * 1000 / e_slx;       % km

    % Script-model reference (same physics, 1 Hz discrete)
    p = struct('total_mass_kg', T.total_mass_kg(i), 'CdA', T.CdA(i), ...
               'Crr', T.Crr(i), 'rho', T.rho(i), 'eta_drv', T.eta_drv(i), ...
               'usable_kwh', T.usable_kwh(i));
    s = simulate_range(p);

    v_fam(i)   = string(T.family(i));
    v_donor(i) = string(T.donor(i));
    v_rLB(i)   = T.range_lb_km(i);             % framework (conservative)
    v_rScr(i)  = s.range_sim_km;               % script model
    v_rSlx(i)  = rng_slx;                      % Simulink model
    v_pct(i)   = 100*(rng_slx - s.range_sim_km)/s.range_sim_km;
    v_floor(i) = T.range_lb_km(i) <= rng_slx;
    v_Ppk(i)   = s.P_peak_kW;                  % peak wheel power (kW)
    v_Tpk(i)   = s.T_wheel_peak_Nm;            % peak wheel torque (Nm)
    v_Pmot(i)  = s.P_motor_req_kW;             % required motor rating (kW)
end

R = table(v_fam, v_donor, v_rLB, v_rScr, v_rSlx, v_pct, v_floor, ...
          v_Ppk, v_Tpk, v_Pmot, ...
    'VariableNames', {'family','donor','range_LB','range_script', ...
    'range_slx','slx_vs_script_pct','LB_below_slx', ...
    'P_peak_kW','T_wheel_peak_Nm','P_motor_req_kW'});

% ── Report ──────────────────────────────────────────────────────────────
fprintf('%-4s %-28s %9s %12s %10s %10s %9s\n', ...
    'Fam','Donor','Range_LB','RangeScript','RangeSLX','SLXvsScr%','LB<=SLX');
fprintf('%s\n', repmat('-', 1, 92));
for i = 1:n
    fprintf('%-4s %-28s %9.1f %12.1f %10.1f %+10.2f %9d\n', ...
        R.family(i), R.donor(i), R.range_LB(i), R.range_script(i), ...
        R.range_slx(i), R.slx_vs_script_pct(i), R.LB_below_slx(i));
end
fprintf('%s\n', repmat('-', 1, 92));

% ── Sizing indicators (peak power, wheel torque, required motor rating) ──
fprintf('\nMotor and torque sizing indicators (from the WMTC cycle):\n');
fprintf('%-4s %-28s %12s %14s %14s\n', ...
    'Fam','Donor','P_peak (kW)','T_wheel (Nm)','P_motor (kW)');
fprintf('%s\n', repmat('-', 1, 76));
for i = 1:n
    fprintf('%-4s %-28s %12.2f %14.0f %14.2f\n', ...
        R.family(i), R.donor(i), R.P_peak_kW(i), ...
        R.T_wheel_peak_Nm(i), R.P_motor_req_kW(i));
end
fprintf('%s\n', repmat('-', 1, 76));

assert(all(R.LB_below_slx), ...
    'VALIDATION FAILURE: Range_LB exceeded the Simulink range in some case.');
assert(all(abs(R.slx_vs_script_pct) < 3.0), ...
    'CONSISTENCY WARNING: Simulink and script models differ by more than 3%%.');

fprintf(['\nPASS: Simulink and script models agree (<3%%), and the framework\n' ...
         'Range_LB is a conservative floor in all %d cases.\n'], n);

writetable(R, 'simulink_validation_results.csv');
fprintf('Saved simulink_validation_results.csv\n');

% Diagram screenshot for the thesis (Figure: model architecture)
print(['-s' mdl], '-dpng', '-r150', 'fig4_simulink_model.png');
fprintf('Saved fig4_simulink_model.png (block-diagram figure for the thesis)\n');
