function [command, next, desired, reference, ttc] = acc_controller(v, vl, gap, present, setSpeed, acceleration, state, p)
%ACC_CONTROLLER Cascaded gap-to-speed controller with discrete PID and AEB.
% State: [mode; integral; previous error; filtered derivative]. SI units.
% Modes: 0 CRUISE, 1 FOLLOW, 2 EMERGENCY_BRAKE.
%#codegen
oldMode = state(1); integral = state(2); previousError = state(3); derivative = state(4);
desired = p.standstill_gap + p.headway*v;
closing = v-vl;
ttc = inf;
if present && closing > 1e-6
    ttc = max(gap,0)/closing;
end
envelope = p.standstill_gap + p.actuator_tau*v + ...
    max(0,v*v/(2*p.brake_max)-vl*vl/(2*p.lead_brake_assumption));
danger = gap <= 2 || (closing > 0.1 && ...
    (ttc < p.ttc_trigger || gap < envelope+p.emergency_margin));
if ~present
    mode = 0;
elseif danger || (oldMode == 2 && (closing > 0.1 || ...
        gap < envelope+p.emergency_margin+3 || (vl < 0.5 && v < 0.5)))
    mode = 2;
elseif oldMode == 2
    mode = 1;
elseif oldMode == 1 && gap <= max(desired+p.follow_exit_margin,envelope+35)
    mode = 1;
elseif gap <= max(desired+p.follow_enter_margin,envelope+25)
    mode = 1;
else
    mode = 0;
end
reference = setSpeed;
if mode == 1
    reference = max(0,min(setSpeed,vl+p.gap_gain*(gap-desired)));
end
err = reference-v;
if mode ~= oldMode
    integral = 0; previousError = err; derivative = 0;
end
alpha = p.dt/(p.derivative_tau+p.dt);
derivative = derivative + alpha*((err-previousError)/p.dt-derivative);
raw = p.kp*err + p.ki*integral + p.kd*derivative;
command = min(p.accel_max,max(-p.brake_max,raw));
step = p.jerk_limit*p.actuator_tau;
command = min(acceleration+step,max(acceleration-step,command));
command = min(p.accel_max,max(-p.brake_max,command));
if mode == 2
    command = -p.brake_max; integral = 0; derivative = 0;
elseif abs(raw-command) < 1e-9 || err*(raw-command) <= 0
    integral = min(p.integral_limit,max(-p.integral_limit,integral+err*p.dt));
end
next = [mode; integral; err; derivative];
end
