# Drive-Cycle Validation of a Retrofit-as-a-Service Framework for Electric Motorcycle Conversion

This repository contains the simulation models, parameters, and results that
support the findings of the paper:

> A. Alqedra, S. F. Toha, et al., "Simulink-Based Powertrain Validation of an
> Archetype-Driven Electric Motorcycle Retrofit Recommendation Framework,"
> International Journal of Power Electronics and Drive Systems (IJPEDS),
> (under review).

The work validates a data-driven Retrofit-as-a-Service (RaaS) recommendation
framework for converting internal combustion engine motorcycles to electric
motorcycles. A longitudinal powertrain model is run over a representative World
Motorcycle Test Cycle (WMTC) to check, for five retrofit archetypes (F1 to F5),
whether the framework's conservative lower-bound range estimate holds against
dynamic drive-cycle operation, and to report motor and torque sizing indicators.

## Repository structure

```
.
|-- matlab/        MATLAB and Simulink source
|   |-- wmtc_cycle.m              Representative WMTC drive cycle (speed vs time)
|   |-- simulate_range.m          Longitudinal model: energy, range, sizing outputs
|   |-- run_comparison.m          Script driver: comparison table and figures 1-3
|   |-- build_simulink_model.m    Builds the Simulink block diagram (.slx)
|   |-- run_simulink_validation.m Runs the Simulink model and cross-checks it
|-- data/
|   |-- raas_params.csv                  Parameters for the five archetype cases
|   |-- expected_results_reference.txt   Numbers a correct run should reproduce
|   |-- comparison_results.csv           Output of run_comparison.m
|   |-- simulink_validation_results.csv  Output of run_simulink_validation.m
|-- figures/
|   |-- fig1_wmtc_cycle.png
|   |-- fig2_range_comparison.png
|   |-- fig3_energy_intensity.png
|   |-- fig4_simulink_model.png
|-- LICENSE
|-- README.md
```

## Requirements

- MATLAB (any recent release) for the script model (`run_comparison.m`).
- Simulink (no additional toolboxes) for the block-diagram model.
- The script model also runs in GNU Octave with no changes.

## How to reproduce the results

Place the contents of `matlab/` and `data/raas_params.csv` in the same working
folder, then run in MATLAB, in this order:

```matlab
>> run_comparison          % script model: comparison table + figures 1-3
>> build_simulink_model    % builds raas_powertrain_model.slx (run once)
>> run_simulink_validation % Simulink model: validation table + figure 4
```

The printed tables and the saved CSVs should match
`data/expected_results_reference.txt`. The Simulink and script implementations
agree to within about 0.2 percent, and the framework lower-bound range is below
the simulated range in all five archetypes.

## The model

Longitudinal road-load model at 1 Hz over the WMTC cycle:

```
F_roll  = m g C_rr
F_aero  = 0.5 rho C_dA v^2
F_inert = m a
F_grade = m g sin(theta)        (theta = 0 for the flat WMTC results)
F_trac  = F_roll + F_aero + F_inert + F_grade
P_wheel = F_trac v
```

Battery-side power applies the drivetrain efficiency on traction and recovers
braking energy at a regenerative efficiency. Energy and distance are integrated
over the cycle to give energy intensity (Wh/km) and range. The model also
reports peak wheel power, peak wheel torque, and a required motor rating.

Fixed parameters: C_rr = 0.015, C_dA = 0.45, eta_drv = 0.85, rho = 1.225 kg/m^3,
usable-energy fraction 0.85, regenerative efficiency 0.35, rider mass 75 kg,
drive-wheel radius 0.28 m. The cycle is deterministic and reproducible.

## Data source

The motorcycle archetypes and tier parameters derive from a clustering of
publicly listed motorcycle specifications (model years 2021 to 2025, up to
900 cc), described in the prior work cited in the paper. The parameter file
`data/raas_params.csv` provides the per-archetype inputs needed to reproduce the
validation; it is the direct input to the simulation.

## Citation

If you use this code, please cite the paper above and this repository.

## License

Released under the MIT License. See `LICENSE`.
