function build_simulink_model()
% BUILD_SIMULINK_MODEL  Programmatically constructs the RaaS powertrain
% Simulink block diagram (raas_powertrain_model.slx).
%
% Run this ONCE in MATLAB (from this folder):
%   >> build_simulink_model
%
% It creates and saves 'raas_powertrain_model.slx', a longitudinal
% vehicle-dynamics model of the retrofitted electric motorcycle:
%
%   [Drive Cycle v(t)] --+--> [v^2] -> [0.5*rho*CdA] --\
%                        |                              +--> [Sum F_trac]
%   [Drive Cycle a(t)] --|--> [x mass]  ---------------/        |
%                        |                                      v
%   [Const m*g*Crr] -----+----------------------------> [Product P_wheel]
%                        |                                      |
%                        |          [1/eta_drv] <-- traction ---+--- braking --> [eta_regen]
%                        |                   \                  |               /
%                        |                    +---> [Switch P_batt] <----------+
%                        |                                      |
%                        |                          [Integrator] -> [1/3600] -> E_Wh
%                        +--> [Integrator] -> [1/1000] -> dist_km
%
% All block parameters reference BASE WORKSPACE VARIABLES (mass, Crr, CdA,
% rho, eta_drv, eta_regen), so the same diagram simulates any motorcycle
% by changing the inputs. Use run_simulink_validation.m to run all five
% representative cases and compare with the script model and the framework.
%
% Requires: Simulink (any recent release). No additional toolboxes.

mdl = 'raas_powertrain_model';

% Start clean
if bdIsLoaded(mdl); close_system(mdl, 0); end
if exist([mdl '.slx'], 'file'); delete([mdl '.slx']); end

new_system(mdl);
open_system(mdl);

% ── Source blocks ──────────────────────────────────────────────────────
add_block('simulink/Sources/From Workspace', [mdl '/v_in'], ...
    'VariableName', 'v_ts', 'SampleTime', '0', ...
    'Position', [40 60 130 90]);                    % speed v (m/s)

add_block('simulink/Sources/From Workspace', [mdl '/a_in'], ...
    'VariableName', 'a_ts', 'SampleTime', '0', ...
    'Position', [40 180 130 210]);                  % acceleration a (m/s^2)

add_block('simulink/Sources/Constant', [mdl '/Froll'], ...
    'Value', 'mass*9.81*Crr', ...
    'Position', [40 300 150 330]);                  % rolling force (N)

% ── Road-load force blocks ─────────────────────────────────────────────
add_block('simulink/Math Operations/Math Function', [mdl '/vsq'], ...
    'Operator', 'square', 'Position', [200 60 250 90]);

add_block('simulink/Math Operations/Gain', [mdl '/AeroGain'], ...
    'Gain', '0.5*rho*CdA', 'Position', [290 60 380 90]);   % F_aero (N)

add_block('simulink/Math Operations/Gain', [mdl '/InertGain'], ...
    'Gain', 'mass', 'Position', [200 180 280 210]);        % F_inert (N)

add_block('simulink/Math Operations/Sum', [mdl '/Ftrac'], ...
    'Inputs', '++++', 'IconShape', 'rectangular', ...
    'Position', [440 150 480 260]);                        % total force (N)

% Grade force: m*g*sin(theta). theta = 0 (flat) for the WMTC results in the
% paper; the block is present so hilly-terrain cases can be studied later.
add_block('simulink/Sources/Constant', [mdl '/Fgrade'], ...
    'Value', 'mass*9.81*sin(grade_rad)', ...
    'Position', [40 380 170 410]);

% ── Wheel power ────────────────────────────────────────────────────────
add_block('simulink/Math Operations/Product', [mdl '/Pwheel'], ...
    'Position', [540 100 580 140]);                        % P = F*v (W)

% Wheel torque T = F_trac * R_wheel, and its running peak (sizing indicator)
add_block('simulink/Math Operations/Gain', [mdl '/WheelR'], ...
    'Gain', 'wheel_radius_m', 'Position', [540 300 600 330]);   % T_wheel (Nm)
add_block('simulink/Math Operations/MinMax', [mdl '/TpeakHold'], ...
    'Function', 'max', 'Position', [640 300 690 330]);
add_block('simulink/Sinks/To Workspace', [mdl '/T_wheel_peak'], ...
    'VariableName', 'T_wheel_peak', 'SaveFormat', 'Array', ...
    'Position', [740 300 840 330]);

% Peak wheel power (sizing indicator) via running max of P_wheel
add_block('simulink/Math Operations/MinMax', [mdl '/PpeakHold'], ...
    'Function', 'max', 'Position', [600 -40 650 -10]);
add_block('simulink/Sinks/To Workspace', [mdl '/P_peak'], ...
    'VariableName', 'P_peak', 'SaveFormat', 'Array', ...
    'Position', [700 -45 800 -15]);

% ── Battery-side power: traction vs regen ──────────────────────────────
add_block('simulink/Math Operations/Gain', [mdl '/TracEff'], ...
    'Gain', '1/eta_drv', 'Position', [640 40 720 70]);     % P/eta when driving

add_block('simulink/Math Operations/Gain', [mdl '/RegenEff'], ...
    'Gain', 'eta_regen', 'Position', [640 180 720 210]);   % P*eta_r when braking

add_block('simulink/Signal Routing/Switch', [mdl '/BattSwitch'], ...
    'Criteria', 'u2 >= Threshold', 'Threshold', '0', ...
    'Position', [780 92 820 148]);                          % P_batt (W)

% ── Energy and distance integration ───────────────────────────────────
add_block('simulink/Continuous/Integrator', [mdl '/EnergyInt'], ...
    'Position', [880 100 920 140]);                         % Joules

add_block('simulink/Math Operations/Gain', [mdl '/J2Wh'], ...
    'Gain', '1/3600', 'Position', [960 100 1030 140]);      % J -> Wh

add_block('simulink/Sinks/To Workspace', [mdl '/E_Wh'], ...
    'VariableName', 'E_Wh', 'SaveFormat', 'Array', ...
    'Position', [1080 100 1160 140]);

add_block('simulink/Sinks/Display', [mdl '/E display'], ...
    'Position', [1080 170 1160 200]);

add_block('simulink/Continuous/Integrator', [mdl '/DistInt'], ...
    'Position', [200 380 240 420]);                          % meters

add_block('simulink/Math Operations/Gain', [mdl '/m2km'], ...
    'Gain', '1/1000', 'Position', [280 380 350 420]);

add_block('simulink/Sinks/To Workspace', [mdl '/dist_km'], ...
    'VariableName', 'dist_km', 'SaveFormat', 'Array', ...
    'Position', [400 380 480 420]);

add_block('simulink/Sinks/Display', [mdl '/D display'], ...
    'Position', [400 450 480 480]);

add_block('simulink/Sinks/Scope', [mdl '/Power scope'], ...
    'Position', [880 200 920 240]);

% ── Wiring ─────────────────────────────────────────────────────────────
AR = {'autorouting', 'on'};
add_line(mdl, 'v_in/1',      'vsq/1',        AR{:});
add_line(mdl, 'vsq/1',       'AeroGain/1',   AR{:});
add_line(mdl, 'a_in/1',      'InertGain/1',  AR{:});
add_line(mdl, 'Froll/1',     'Ftrac/1',      AR{:});
add_line(mdl, 'AeroGain/1',  'Ftrac/2',      AR{:});
add_line(mdl, 'InertGain/1', 'Ftrac/3',      AR{:});
add_line(mdl, 'Fgrade/1',    'Ftrac/4',      AR{:});
add_line(mdl, 'Ftrac/1',     'Pwheel/1',     AR{:});
add_line(mdl, 'Ftrac/1',     'WheelR/1',     AR{:});
add_line(mdl, 'WheelR/1',    'TpeakHold/1',  AR{:});
add_line(mdl, 'TpeakHold/1', 'T_wheel_peak/1', AR{:});
add_line(mdl, 'Pwheel/1',    'PpeakHold/1',  AR{:});
add_line(mdl, 'PpeakHold/1', 'P_peak/1',     AR{:});
add_line(mdl, 'v_in/1',      'Pwheel/2',     AR{:});
add_line(mdl, 'Pwheel/1',    'TracEff/1',    AR{:});
add_line(mdl, 'Pwheel/1',    'RegenEff/1',   AR{:});
add_line(mdl, 'TracEff/1',   'BattSwitch/1', AR{:});
add_line(mdl, 'Pwheel/1',    'BattSwitch/2', AR{:});   % switch control
add_line(mdl, 'RegenEff/1',  'BattSwitch/3', AR{:});
add_line(mdl, 'BattSwitch/1','EnergyInt/1',  AR{:});
add_line(mdl, 'EnergyInt/1', 'J2Wh/1',       AR{:});
add_line(mdl, 'J2Wh/1',      'E_Wh/1',       AR{:});
add_line(mdl, 'J2Wh/1',      'E display/1',  AR{:});
add_line(mdl, 'BattSwitch/1','Power scope/1',AR{:});
add_line(mdl, 'v_in/1',      'DistInt/1',    AR{:});
add_line(mdl, 'DistInt/1',   'm2km/1',       AR{:});
add_line(mdl, 'm2km/1',      'dist_km/1',    AR{:});
add_line(mdl, 'm2km/1',      'D display/1',  AR{:});

% ── Label key signals so the diagram reads clearly in a figure caption ──
label_signal(mdl, 'v_in/1',       'v (m/s)');
label_signal(mdl, 'a_in/1',       'a (m/s^2)');
label_signal(mdl, 'AeroGain/1',   'F_aero');
label_signal(mdl, 'Froll/1',      'F_roll');
label_signal(mdl, 'InertGain/1',  'F_inert');
label_signal(mdl, 'Ftrac/1',      'F_trac');
label_signal(mdl, 'Pwheel/1',     'P_wheel');
label_signal(mdl, 'WheelR/1',     'T_wheel');
label_signal(mdl, 'BattSwitch/1', 'P_batt');

% ── Annotation and solver configuration ────────────────────────────────
add_block('built-in/Note', [mdl '/RaaS Retrofit Powertrain Model — ' ...
    'Longitudinal dynamics over WMTC cycle (IIUM MSc Mechatronics)'], ...
    'Position', [40 20 60 30]);

set_param(mdl, 'Solver', 'ode45', 'StopTime', 't_end', ...
    'MaxStep', '0.5', 'SaveTime', 'on');

save_system(mdl);
fprintf('Created and saved %s.slx\n', mdl);
fprintf('Next: run run_simulink_validation.m to simulate all five cases.\n');
end


function label_signal(mdl, port, name)
% Attach a name to the signal line leaving the given source port, so the
% wire is labelled in the diagram (improves readability for publication).
    try
        parts = split(port, '/');
        blk = strjoin(parts(1:end-1), '/');
        pnum = str2double(parts{end});
        lh = get_param([mdl '/' blk], 'LineHandles');
        if pnum <= numel(lh.Outport) && lh.Outport(pnum) > 0
            set_param(lh.Outport(pnum), 'Name', name);
        end
    catch
        % labelling is cosmetic; ignore if a handle is unavailable
    end
end
