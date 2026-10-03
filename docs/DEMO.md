# Demonstration and interview notes

## Five-minute demonstration

1. Explain the inputs: ego/lead speed, bumper gap, set speed, detection flag, and acceleration feedback. Show the 5 m + 1.5 s × speed gap policy.
2. Run `addpath('matlab'); run_project`. Open the slowdown plot and point out lead deceleration from 10–16 s, ego response, and gap convergence.
3. Run `build_simulink_model; open_system('acc_aeb')`. Trace the controller → plant → state-delay feedback path. The scenario source supplies lead acceleration, presence, and set speed.
4. Show the hard-braking result: AEB override, actuator lag, saturated −4 m/s² command, 7.93 m remaining clearance, and the temporary desired-gap violation.
5. Show the headway and P/PI/PID comparisons. Explain why the 1.0 s setting can conflict with the AEB envelope and why PID does not win every metric.
6. Open the CI run and its `.slx`/CSV artifacts to show reproducibility. Mention the infeasible collision test and the limits of the plant abstraction.

## Changing an experiment

Edit the JSON and rerun both implementations. Scenario speeds are in m/s; divide km/h by 3.6. `events` holds `[time_seconds, lead_acceleration_mps2]` pairs in ascending order. Acceleration remains active until the next event. Add `[time, 0]` to end braking before standstill. `lead_present_until` specifies lane departure; a value beyond the duration keeps the lead detected.

The controller's `headway`, PID gains, lag, and limits are shared between MATLAB/Python. Rebuild the Simulink model after changing parameters: its MATLAB Function block embeds numeric configuration for a self-contained model workspace. Core `.m` files must remain on the MATLAB path when running the generated model.

For fresh cross-language fixtures after changes:

```bash
python python/run_experiments.py --output results/parity
```

Then run `test_project; run_project; run_simulink` in MATLAB. `test_project` prefers fresh `results/parity` traces, falling back to committed `results/traces` if they are absent.

## Questions to be ready for

- **Why a cascaded controller?** The gap loop maps metres of spacing error into a speed reference; a single inner speed loop then controls the acceleration plant in both cruise and follow modes.
- **Why anti-windup?** A speed error can persist during saturation; unrestricted integral accumulation would delay recovery and increase overshoot.
- **Why filter the derivative?** Differencing sampled error amplifies rapid changes and would amplify sensor noise. This experiment uses ideal sensing, so it does not quantify noise robustness.
- **Why not claim guaranteed safe distance?** Actuation is limited, the lead may brake harder, initial conditions can be infeasible, and the switching rule is heuristic. Report gap violations and collisions explicitly.
- **What does the plant omit?** Tire forces, drag, grade, drivetrain, brake hydraulics, sensing latency/noise, target association, and vehicle-to-vehicle geometry beyond bumper gap.
- **What would you improve first?** Jointly design the headway target and braking envelope; then add sensing delay and varying braking authority. Reduce the artificial standstill jerk before making comfort claims.

## Résumé wording grounded in this repository

**Adaptive Cruise Control & Emergency Braking | MATLAB, Simulink, Python**

- Implemented a cascaded PID ACC controller with cruise, follow, and emergency-braking modes, time-headway spacing, actuator saturation, anti-windup, and first-order acceleration dynamics.
- Evaluated six driving scenarios and three headway settings; baseline hard braking from an 80 km/h lead trajectory retained 7.93 m simulated clearance with a −4 m/s² ego braking limit.
- Compared P/PI/PID response and automated behavioral and cross-implementation validation, including an infeasible-gap collision test.

Use these claims only after running the project and understanding the code and plots. This project does not establish experience deploying a production ADAS controller or tuning a real vehicle.
