"""Deterministic reference for the MATLAB/Simulink ACC. All quantities are SI."""
from pathlib import Path
import json
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
CRUISE, FOLLOW, EMERGENCY_BRAKE = 0, 1, 2
COLUMNS = ["time", "ego_speed", "lead_speed", "gap", "desired_gap", "command",
           "acceleration", "mode", "lead_present", "reference_speed", "ttc"]


def inputs():
    return (json.loads((ROOT / "config/parameters.json").read_text()),
            json.loads((ROOT / "config/scenarios.json").read_text()))


def controller(v, vl, gap, present, set_speed, acceleration, state, p):
    """One sample; state = [mode, integral, previous error, filtered derivative]."""
    old_mode, integral, previous_error, derivative = state
    desired = p["standstill_gap"] + p["headway"] * v
    closing = v - vl
    ttc = max(gap, 0) / closing if present and closing > 1e-6 else np.inf
    envelope = (p["standstill_gap"] + p["actuator_tau"] * v
                + max(0, v*v / (2*p["brake_max"])
                      - vl*vl / (2*p["lead_brake_assumption"])))
    danger = (gap <= 2 or (closing > 0.1 and
              (ttc < p["ttc_trigger"] or gap < envelope + p["emergency_margin"])))
    if not present:
        mode = CRUISE
    elif danger or (old_mode == EMERGENCY_BRAKE and
                   (closing > 0.1 or gap < envelope + p["emergency_margin"] + 3
                    or (vl < 0.5 and v < 0.5))):
        mode = EMERGENCY_BRAKE
    elif old_mode == EMERGENCY_BRAKE:
        mode = FOLLOW
    elif old_mode == FOLLOW and gap <= max(desired + p["follow_exit_margin"], envelope + 35):
        mode = FOLLOW
    elif gap <= max(desired + p["follow_enter_margin"], envelope + 25):
        mode = FOLLOW
    else:
        mode = CRUISE
    reference = set_speed
    if mode == FOLLOW:
        reference = max(0, min(set_speed, vl + p["gap_gain"] * (gap - desired)))
    error = reference - v
    if mode != old_mode:
        integral, previous_error, derivative = 0.0, error, 0.0
    alpha = p["dt"] / (p["derivative_tau"] + p["dt"])
    derivative += alpha * ((error - previous_error) / p["dt"] - derivative)
    raw = p["kp"]*error + p["ki"]*integral + p["kd"]*derivative
    command = min(p["accel_max"], max(-p["brake_max"], raw))
    # Limit command relative to measured acceleration; emergency bypasses comfort.
    step = p["jerk_limit"] * p["actuator_tau"]
    command = min(acceleration + step, max(acceleration - step, command))
    command = min(p["accel_max"], max(-p["brake_max"], command))
    if mode == EMERGENCY_BRAKE:
        command, integral, derivative = -p["brake_max"], 0.0, 0.0
    elif abs(raw - command) < 1e-9 or error * (raw - command) <= 0:
        integral = min(p["integral_limit"], max(-p["integral_limit"],
                       integral + error * p["dt"]))
    return command, np.array([mode, integral, error, derivative]), desired, reference, ttc


def advance(speed, acceleration, dt):
    """Constant-acceleration kinematics with exact stopping within a sample."""
    moving_time = dt
    if acceleration < 0:
        moving_time = min(dt, speed / -acceleration)
    distance = speed * moving_time + 0.5 * acceleration * moving_time**2
    return max(0, speed + acceleration * dt), distance


def simulate(s, p):
    n = round(s["duration"] / p["dt"]) + 1
    trace = np.zeros((n, len(COLUMNS)))
    v, vl, gap, a = s["ego_speed"], s["lead_speed"], s["initial_gap"], 0.0
    state = np.array([CRUISE, 0., s["set_speed"] - v, 0.])
    for k in range(n):
        t = k * p["dt"]
        present = t < s["lead_present_until"]
        u, state, desired, reference, ttc = controller(v, vl, gap, present,
                                                       s["set_speed"], a, state, p)
        trace[k] = [t, v, vl, gap, desired, u, a, state[0], present, reference, ttc]
        lead_a = s["events"][0][1]
        for event_t, event_a in s["events"]:
            if t >= event_t - 1e-9:
                lead_a = event_a
        next_a = a + p["dt"] / p["actuator_tau"] * (u - a)
        v, dx = advance(v, next_a, p["dt"])
        vl, dl = advance(vl, lead_a, p["dt"])
        gap += dl - dx  # Never clamp gap: negative means collision/overlap.
        a = 0.0 if v == 0 and next_a < 0 else next_a
    return trace


def metrics(trace, s, p):
    t, v, vl, gap, desired, u, a, mode, present, ref, ttc = trace.T
    active = present.astype(bool)
    follow = active & (mode == FOLLOW)
    post = t >= s["settle_after"] - 1e-9
    within = abs(v - s["final_speed"]) <= 0.5
    bad = np.flatnonzero(post & ~within)
    start = np.flatnonzero(post)[0] if post.any() else len(t)
    settled_at = (bad[-1] + 1) if len(bad) else start
    # Require at least five final seconds inside the tolerance.
    settling = (t[settled_at] - s["settle_after"]
                if settled_at < len(t) and t[-1] - t[settled_at] >= 5 else None)
    collision = np.flatnonzero(active & (gap <= 0))
    jerk = np.diff(a) / p["dt"]
    return {
        "scenario": s["name"], "headway_s": p["headway"],
        "min_gap_m": float(gap[active].min()) if active.any() else None,
        "min_gap_margin_m": float((gap-desired)[active].min()) if active.any() else None,
        "below_desired_gap_s": float(np.count_nonzero(active[:-1] & (gap[:-1] < desired[:-1]))*p["dt"]),
        "follow_gap_rmse_m": float(np.sqrt(np.mean((gap[follow]-desired[follow])**2))) if follow.any() else None,
        "speed_settling_s": None if settling is None else float(settling),
        "set_speed_overshoot_kmh": float(max(0, v.max()-s["set_speed"])*3.6),
        "peak_acceleration_mps2": float(a.max()), "peak_braking_mps2": float(-a.min()),
        "rms_jerk_mps3": float(np.sqrt(np.mean(jerk**2))),
        "peak_jerk_mps3": float(np.max(abs(jerk))),
        "emergency_s": float(np.count_nonzero(mode[:-1] == EMERGENCY_BRAKE)*p["dt"]),
        "mode_transitions": int(np.count_nonzero(np.diff(mode))),
        "collision": bool(len(collision)),
        "first_collision_s": float(t[collision[0]]) if len(collision) else None,
        "final_speed_kmh": float(v[-1]*3.6)
    }
