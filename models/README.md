# Generated Simulink model

Run `addpath('matlab'); build_simulink_model` from the repository root in MATLAB + Simulink. This creates `acc_aeb.slx`. Open it with `open_system('acc_aeb')` and press Run for the lead-slowdown scenario, or call `run_simulink` for all scenarios with parity checks.

The model is also built in GitHub Actions and uploaded in the `matlab-simulink-results` artifact. Keep the repository's `matlab/` directory on the MATLAB path: the MATLAB Function blocks call the shared core functions. Rebuild after modifying configuration.
