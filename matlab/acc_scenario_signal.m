function signal = acc_scenario_signal(s,p)
% Columns: time, lead acceleration, lead present, driver set speed.
t = (0:round(s.duration/p.dt))'*p.dt;
a = zeros(size(t));
for j = 1:size(s.events,1)
    a(t >= s.events(j,1)-1e-9) = s.events(j,2);
end
signal = [t,a,double(t < s.lead_present_until),repmat(s.set_speed,size(t))];
end
