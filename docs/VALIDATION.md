# Validation record

Verified on **3 October 2026**, using Python 3.11 and **MATLAB + Simulink R2024b** on GitHub-hosted Ubuntu runners.

- [Successful validation run](https://github.com/tanishy7777/adaptive-cruise-control/actions/runs/37142122403)
- Tested source commit: `ec9e2f8` (later packaging/documentation changes do not change the controller or experiment code).
- Python: **12 tests passed**, including 18 scenario/headway combinations, emergency hold/release, tracking, bounds, stopping, anti-windup, infeasible collision detection, and step-size sensitivity.
- MATLAB: **18 scenario/headway checks plus edge cases passed**; all six baseline traces matched Python.
- Simulink: model built successfully and **all six baseline traces matched MATLAB**.

| Simulink scenario | Maximum absolute difference, columns 1–10 |
|---|---:|
| free_cruise | 0 |
| steady_follow | 0 |
| lead_slowdown | 0 |
| hard_brake | 0 |
| catch_slower | 0 |
| road_clears | 0 |

The Simulink comparison requires <10⁻⁷ absolute difference for time, speed, gap, desired gap, command, actual acceleration, mode, lead presence, and reference speed. TTC requires an identical infinity mask and <10⁻⁶ difference for finite values. Python/MATLAB tolerances are 10⁻⁶ and 10⁻⁴ respectively to accommodate the CSV's ten significant figures.

The downloaded MATLAB metric tables were also compared with Python locally: baseline, headway, and controller tables all agreed within 1.3×10⁻¹² across numeric fields.

`models/acc_aeb.slx` is the artifact downloaded from this run. The runner executed in headless mode; Scope UI/diagram printing was unavailable, but model compilation, simulation, CSV logging, and all parity checks completed. MATLAB scenario plots were generated successfully. Open the model in a MATLAB desktop to use its four-panel Scope.

Model SHA-256: `1d4f771173211840a83989c712b3b19a906eb2dad4f4d15164dab03cf05d7ac5`.

The validation establishes agreement and scenario behavior for this simplified model. It does not establish performance on a real vehicle, formal collision avoidance, or robustness to sensing and tire/road uncertainty. See [design assumptions](DESIGN.md) and [observed limitations](RESULTS.md).
