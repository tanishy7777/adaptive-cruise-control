function trace = acc_simulate(s,p)
%ACC_SIMULATE Sample -> control -> log -> actuator/kinematic update.
signal = acc_scenario_signal(s,p);
vehicle = [s.ego_speed;s.lead_speed;s.initial_gap;0];
state = [0;0;s.set_speed-s.ego_speed;0];
trace = zeros(size(signal,1),11);
for k = 1:size(signal,1)
    [u,state,desired,reference,ttc] = acc_controller(vehicle(1),vehicle(2),...
        vehicle(3),signal(k,3),signal(k,4),vehicle(4),state,p);
    trace(k,:) = [signal(k,1),vehicle(1:3)',desired,u,vehicle(4),state(1),...
        signal(k,3),reference,ttc];
    vehicle = acc_plant(vehicle,u,signal(k,2),p.dt,p.actuator_tau);
end
end
