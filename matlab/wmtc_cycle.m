function [t, v_kmh] = wmtc_cycle()
% WMTC_CYCLE  Representative motorcycle drive cycle (WMTC-style).
%
%   [t, v_kmh] = WMTC_CYCLE() returns a time vector t (seconds, 1 Hz) and the
%   corresponding vehicle speed v_kmh (km/h) for a representative urban-to-
%   rural motorcycle driving cycle.
%
%   This is a compact, reproducible approximation of the UNECE World
%   Motorcycle Test Cycle (WMTC) Part 1 + Part 2 profile, suitable for
%   class 1 and class 2 motorcycles (engine displacement up to ~900 cc).
%   It captures the key features that a steady-state cruise model omits:
%   repeated acceleration, deceleration, idle, and a sustained higher-speed
%   rural segment. The cycle is built from standard WMTC micro-trip
%   segments (accelerate, cruise, decelerate, idle) so that the integrated
%   energy reflects realistic transient demand.
%
%   Reference:
%     UNECE Global Technical Regulation No. 2 (WMTC), Addendum 2,
%     "Measurement procedure for two-wheeled motorcycles equipped with a
%     positive or compression ignition engine."
%
%   Total duration ~ 600 s. Speeds are non-negative.
%
%   Author: RaaS thesis project, IIUM. Reproducible (deterministic).

    % Build the cycle as a sequence of [duration_s, start_kmh, end_kmh]
    % micro-trips. Linear interpolation within each segment at 1 Hz.
    % Profile: urban (low speed, frequent stops) -> rural (sustained cruise).

    segments = [ ...
        % dur   v0    v1     description
         5,     0,    0;   ... % idle
         12,    0,   30;   ... % urban accel
         15,   30,   30;   ... % urban cruise
         8,    30,   15;   ... % decel
         10,   15,   40;   ... % accel
         18,   40,   40;   ... % cruise
         9,    40,    0;   ... % decel to stop
         6,     0,    0;   ... % idle
         14,    0,   45;   ... % accel
         20,   45,   45;   ... % cruise
         10,   45,   25;   ... % decel
         12,   25,   50;   ... % accel
         22,   50,   50;   ... % cruise
         11,   50,    0;   ... % decel to stop
         7,     0,    0;   ... % idle
         % --- rural / higher-speed part ---
         18,    0,   60;   ... % strong accel
         30,   60,   60;   ... % rural cruise
         12,   60,   45;   ... % decel
         15,   45,   75;   ... % accel to high speed
         40,   75,   75;   ... % sustained high cruise
         14,   75,   55;   ... % decel
         20,   55,   80;   ... % accel to top
         45,   80,   80;   ... % sustained top cruise
         16,   80,   50;   ... % decel
         25,   50,   65;   ... % accel
         35,   65,   65;   ... % cruise
         18,   65,    0;   ... % long decel to stop
         8,     0,    0    ... % idle
    ];

    t = [];
    v_kmh = [];
    t_now = 0;

    for i = 1:size(segments, 1)
        dur = segments(i, 1);
        v0  = segments(i, 2);
        v1  = segments(i, 3);
        n   = round(dur);                 % 1 Hz sampling
        seg_t = (t_now + (1:n))';          % column vector
        seg_v = (v0 + (v1 - v0) * (1:n) / n)';
        t = [t; seg_t]; %#ok<AGROW>
        v_kmh = [v_kmh; seg_v]; %#ok<AGROW>
        t_now = t_now + n;
    end

    % Ensure non-negative speed
    v_kmh = max(v_kmh, 0);
end
