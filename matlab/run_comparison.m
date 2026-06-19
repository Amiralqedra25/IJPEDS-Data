% run_comparison.m
% =================
% RaaS Framework vs Simulink/MATLAB Longitudinal Simulation
% MSc Thesis: AI-Enhanced Retrofit-as-a-Service (RaaS), IIUM
%
% This is the MAIN script for the journal-paper parallel validation.
% It reads the five representative cases produced by the Python RaaS
% framework (raas_params.csv), runs a dynamic longitudinal simulation over
% the WMTC drive cycle for each, and compares:
%   - framework conservative estimate   Range_LB   (steady-state)
%   - simulated range                   Range_sim  (dynamic WMTC)
%
% Outputs:
%   - Console table of the comparison
%   - comparison_results.csv  (for the paper)
%   - Figure 1: WMTC drive cycle
%   - Figure 2: Range_LB vs Range_sim bar chart per family
%   - Figure 3: energy intensity (Wh/km) framework vs simulated
%
% USAGE (in MATLAB, from the matlab/ folder):
%   >> run_comparison
%
% Requires: wmtc_cycle.m, simulate_range.m, raas_params.csv (same folder)
% No special toolbox required (base MATLAB only).

clear; clc; close all;

%% 1. Load framework parameters
T = readtable('raas_params.csv');
nCases = height(T);
fprintf('Loaded %d representative cases from raas_params.csv\n\n', nCases);

%% 2. Run dynamic simulation for each case
% Preallocate plain arrays (avoids the "added rows to table" warning that
% occurs when a table is grown one cell at a time inside a loop).
fam_c   = strings(nCases,1);
donor_c = strings(nCases,1);
vref_c  = zeros(nCases,1);
use_c   = zeros(nCases,1);
eF_c    = zeros(nCases,1);
eS_c    = zeros(nCases,1);
rLB_c   = zeros(nCases,1);
rSim_c  = zeros(nCases,1);
dkm_c   = zeros(nCases,1);
dpct_c  = zeros(nCases,1);
cyc_t = []; cyc_v = [];

for i = 1:nCases
    p = struct();
    p.total_mass_kg = T.total_mass_kg(i);
    p.CdA           = T.CdA(i);
    p.Crr           = T.Crr(i);
    p.rho           = T.rho(i);
    p.eta_drv       = T.eta_drv(i);
    p.usable_kwh    = T.usable_kwh(i);

    sim = simulate_range(p);

    fam_c(i)   = string(T.family(i));
    donor_c(i) = string(T.donor(i));
    vref_c(i)  = T.v_ref_kmh(i);
    use_c(i)   = T.usable_kwh(i);
    eF_c(i)    = T.e_UB_Wh_per_km(i);          % framework intensity (with kappa)
    eS_c(i)    = sim.e_sim_Wh_per_km;          % dynamic intensity
    rLB_c(i)   = T.range_lb_km(i);             % conservative km
    rSim_c(i)  = sim.range_sim_km;             % dynamic km
    dkm_c(i)   = sim.range_sim_km - T.range_lb_km(i);
    dpct_c(i)  = 100 * (sim.range_sim_km - T.range_lb_km(i)) / T.range_lb_km(i);

    if i == 1
        cyc_t = sim.t; cyc_v = sim.v_kmh;
    end
end

% Assemble the results table in one shot (no incremental growth)
results = table(fam_c, donor_c, vref_c, use_c, eF_c, eS_c, ...
                rLB_c, rSim_c, dkm_c, dpct_c, ...
    'VariableNames', {'family','donor','v_ref_kmh','usable_kwh', ...
    'e_framework','e_simulated','range_LB','range_sim','diff_km','diff_pct'});

%% 3. Console comparison table
fprintf('%-4s %-28s %8s %10s %10s %10s %10s %8s\n', ...
    'Fam','Donor','v_ref','e_frame','e_sim','Range_LB','Range_sim','Diff%');
fprintf('%s\n', repmat('-', 1, 96));
for i = 1:nCases
    fprintf('%-4s %-28s %8.0f %10.1f %10.1f %10.1f %10.1f %+8.1f\n', ...
        results.family(i), results.donor(i), results.v_ref_kmh(i), ...
        results.e_framework(i), results.e_simulated(i), ...
        results.range_LB(i), results.range_sim(i), results.diff_pct(i));
end
fprintf('%s\n', repmat('-', 1, 96));
fprintf('Mean simulated-vs-conservative range difference: %+.1f%%\n\n', ...
    mean(results.diff_pct));

%% 4. Save comparison CSV for the paper
writetable(results, 'comparison_results.csv');
fprintf('Saved comparison_results.csv\n');

%% 5. Figure 1 — WMTC drive cycle
% White background, black axes/text, exported at 200 dpi for publication.
f1 = figure('Name','WMTC Drive Cycle','Color','w','Position',[100 100 760 340]);
ax1 = axes(f1); hold(ax1,'on');
plot(ax1, cyc_t, cyc_v, 'LineWidth', 1.6, 'Color', [0.85 0.33 0.10]);
grid(ax1,'on'); box(ax1,'on');
set(ax1, 'Color','w', 'XColor','k', 'YColor','k', 'GridColor',[0.8 0.8 0.8], ...
    'FontName','Arial', 'FontSize', 11);
xlabel(ax1,'Time (s)','Color','k'); ylabel(ax1,'Speed (km/h)','Color','k');
title(ax1,'Representative WMTC drive cycle used for simulation','Color','k');
xlim(ax1,[0 max(cyc_t)]); ylim(ax1,[0 max(cyc_v)*1.1]);
exportgraphics(f1, 'fig1_wmtc_cycle.png', 'Resolution', 200, 'BackgroundColor','white');

%% 6. Figure 2 — Range comparison bar chart
f2 = figure('Name','Range Comparison','Color','w','Position',[100 100 760 400]);
ax2 = axes(f2);
famLabels = cellstr(results.family);
rangeData = [results.range_LB, results.range_sim];
b = bar(ax2, rangeData, 'grouped');
b(1).FaceColor = [0.30 0.45 0.70];   % framework (blue)
b(2).FaceColor = [0.20 0.65 0.32];   % simulated (green)
set(ax2, 'XTickLabel', famLabels, 'Color','w', 'XColor','k', 'YColor','k', ...
    'GridColor',[0.8 0.8 0.8], 'FontName','Arial', 'FontSize', 11);
grid(ax2,'on'); box(ax2,'on');
ylabel(ax2,'Range (km)','Color','k'); xlabel(ax2,'Retrofit family','Color','k');
lg2 = legend(ax2, {'Framework Range_{LB} (conservative)','Simulated range (WMTC)'}, ...
    'Location','northwest', 'TextColor','k');
set(lg2, 'Color','w', 'EdgeColor',[0.4 0.4 0.4]);
title(ax2,'Conservative framework estimate vs dynamic simulation','Color','k');
% value labels on bars
for k = 1:2
    xt = b(k).XEndPoints; yt = b(k).YEndPoints;
    text(ax2, xt, yt + 4, compose('%.0f', yt'), ...
        'HorizontalAlignment','center', 'FontSize', 9, 'Color','k');
end
ylim(ax2,[0 max(rangeData(:))*1.15]);
exportgraphics(f2, 'fig2_range_comparison.png', 'Resolution', 200, 'BackgroundColor','white');

%% 7. Figure 3 — Energy intensity comparison
f3 = figure('Name','Energy Intensity','Color','w','Position',[100 100 760 400]);
ax3 = axes(f3);
eData = [results.e_framework, results.e_simulated];
b2 = bar(ax3, eData, 'grouped');
b2(1).FaceColor = [0.30 0.45 0.70];   % framework (blue)
b2(2).FaceColor = [0.85 0.33 0.10];   % simulated (orange)
set(ax3, 'XTickLabel', famLabels, 'Color','w', 'XColor','k', 'YColor','k', ...
    'GridColor',[0.8 0.8 0.8], 'FontName','Arial', 'FontSize', 11);
grid(ax3,'on'); box(ax3,'on');
ylabel(ax3,'Energy intensity (Wh/km)','Color','k'); xlabel(ax3,'Retrofit family','Color','k');
lg3 = legend(ax3, {'Framework (with transient factor \kappa)','Simulated (WMTC dynamic)'}, ...
    'Location','northwest', 'TextColor','k');
set(lg3, 'Color','w', 'EdgeColor',[0.4 0.4 0.4]);
title(ax3,'Energy intensity: framework vs dynamic cycle','Color','k');
for k = 1:2
    xt = b2(k).XEndPoints; yt = b2(k).YEndPoints;
    text(ax3, xt, yt + 1, compose('%.1f', yt'), ...
        'HorizontalAlignment','center', 'FontSize', 9, 'Color','k');
end
ylim(ax3,[0 max(eData(:))*1.15]);
exportgraphics(f3, 'fig3_energy_intensity.png', 'Resolution', 200, 'BackgroundColor','white');

fprintf('Saved fig1_wmtc_cycle.png, fig2_range_comparison.png, fig3_energy_intensity.png\n');
fprintf('\nDone. Use comparison_results.csv and the three figures in the paper.\n');
