function next = acc_plant(vehicle, command, leadAcceleration, dt, tau)
%ACC_PLANT Vehicle state: [ego speed; lead speed; bumper gap; ego acceleration].
%#codegen
a = vehicle(4)+dt/tau*(command-vehicle(4));
[v, dx] = acc_advance(vehicle(1),a,dt);
[vl, dl] = acc_advance(vehicle(2),leadAcceleration,dt);
gap = vehicle(3)+dl-dx;
if v == 0 && a < 0
    a = 0;
end
next = [v;vl;gap;a];
end
