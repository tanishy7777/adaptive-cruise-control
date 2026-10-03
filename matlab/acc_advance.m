function [vNext, distance] = acc_advance(v, a, dt)
%ACC_ADVANCE No reverse motion; integrate exactly up to a within-step stop.
%#codegen
movingTime = dt;
if a < 0
    movingTime = min(dt,v/(-a));
end
distance = v*movingTime + 0.5*a*movingTime^2;
vNext = max(0,v+a*dt);
end
