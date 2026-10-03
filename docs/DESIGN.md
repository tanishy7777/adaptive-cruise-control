# Control design

## Signals and conventions

All controller and plant calculations use SI units. Speed plots alone convert m/s to km/h. `gap` is bumper-to-bumper clearance, so no separate vehicle length is subtracted. A positive command accelerates; a negative command brakes. Lead presence is a perfect Boolean signal. At lane departure, the underlying lead trajectory continues but is ignored by the controller and gap metrics.

The sensor/controller input is ego speed, lead speed, gap, lead presence, driver set speed, and measured ego acceleration. The acceleration feedback supports the comfort limiter. Controller memory is `[mode, integral, previous_error, filtered_derivative]`. Vehicle state is `[ego_speed, lead_speed, gap, acceleration]`.

## Cascaded control law

The following controller has an outer proportional gap loop and an inner PID speed loop. It is not two independently switched PID integrators.

$$d^*=5+T_hv,\quad v_r=\min(v_{set},\max(0,v_l+K_g(d-d^*))).$$

The baseline gains are $K_g=0.2\,s^{-1}$, $K_p=0.8\,s^{-1}$, $K_i=0.12\,s^{-2}$, and $K_d=0.08$. With $e_k=v_{r,k}-v_k$:

$$D_k=D_{k-1}+\frac{\Delta t}{\tau_D+\Delta t}\left(\frac{e_k-e_{k-1}}{\Delta t}-D_{k-1}\right),$$

$$u_{raw,k}=K_pe_k+K_iI_k+K_dD_k,\qquad \tau_D=0.2\,s.$$

First saturate to [−4, +2] m/s², then constrain the command to measured acceleration ± $j_{max}\tau_a$, then reapply the physical bounds. With the chosen actuator discretization this bounds the change in moving-vehicle acceleration to $j_{max}\Delta t$, where $j_{max}=2.5$ m/s³. It is not a command slew-rate limit.

Update the integral by $e_k\Delta t$ only when the raw output matches the applied command, or when the error would drive the output back toward the applied command. Clamp the integral to ±10 m/s. Reset integral and derivative memory on mode transitions. Emergency mode freezes/resets that memory and sets the command to −4 m/s².

Gains were selected by iterative scenario checks, not a formal optimization or a claim of global stability. The P/PI/PID study changes only `ki`/`kd` and preserves all supervisor and actuator settings.

## Supervisor

Let $c=v-v_l$ and define a heuristic stopping envelope:

$$d_b=d_0+\tau_av+\max\left(0,\frac{v^2}{2b_e}-\frac{v_l^2}{2b_l}\right),$$

where $b_e=4$ m/s² and assumed lead braking $b_l=6$ m/s². The lag term is a conservative heuristic, not an exact solution of coupled braking trajectories. TTC is $\max(d,0)/c$ for a detected lead and $c>10^{-6}$ m/s; otherwise it is infinite.

Rules are evaluated in this priority order:

1. No detected lead → CRUISE.
2. Gap ≤2 m, or closing >0.1 m/s with either TTC <1.5 s or gap <$d_b+2$ m → EMERGENCY_BRAKE.
3. Already in emergency → remain there while closing >0.1 m/s, gap <$d_b+5$ m, or both vehicle speeds are <0.5 m/s.
4. Released from emergency → FOLLOW for that sample.
5. Already in FOLLOW and gap ≤max($d^*+18$, $d_b+35$) m → retain FOLLOW.
6. Gap ≤max($d^*+8$, $d_b+25$) m → FOLLOW.
7. Otherwise → CRUISE.

The different entry/exit thresholds reduce switching at a boundary. The predictive margins engage following early when relative speeds are high. After stopping, the emergency brake remains commanded while actual acceleration is zero. A departing lead immediately clears the latch; a moving lead must also satisfy the release conditions.

## Plant and sample timing

The controller runs at 50 Hz. Actuator response is a first-order lag discretized by forward Euler:

$$a_{k+1}=a_k+\frac{\Delta t}{0.25}(u_k-a_k).$$

Use $0<\Delta t\leq0.25$ s to retain a convex actuator update; the checked default is 0.02 s. The vehicle holds $a_{k+1}$ over the next interval. Speed and position increments use constant-acceleration kinematics. If braking would cross zero speed, integrate only until the exact stop time within that interval and hold the vehicle at rest. At rest, the realized acceleration is zero even if the brake command remains negative.

At each sample:

1. Read current vehicle state and current scenario inputs.
2. Evaluate supervisor/PID and update controller memory.
3. Log current speeds/gap/realized acceleration together with this sample's command and new mode.
4. Update the actuator, then ego and lead motion, then relative gap.

The lead acceleration profile is zero-order held. It has no actuator lag. Negative lead acceleration persists after stopping but cannot reverse the lead. Gap is **never clamped**; a nonpositive detected gap is a collision. Explicit Unit Delay blocks implement the same sample order in Simulink and break algebraic feedback loops.

The model uses direct acceleration authority, not wheel torque or throttle percentage. Mass is unnecessary for this abstraction. It does not implement $m\dot v=F_{traction}-F_{drag}$.

## Verification rationale

Physics checks cover actuator bounds and nonnegative speed. Scenario checks cover tracking, braking, loss of target, and collisions. Step-size checks compare 20 ms against 10 ms (maximum allowed speed difference 0.3 m/s and gap difference 1 m across baseline scenarios). These are numerical sensitivity checks, not evidence of continuous-time safety.

Cross-language checks compare freshly regenerated Python traces to MATLAB. MATLAB and Simulink use the same core controller/plant functions but different scheduling mechanisms; their parity check catches block wiring, delay, source interpolation, and logging errors. It does not independently validate the physical model.
