function test_project()
%TEST_PROJECT MATLAB behavioral checks and parity with fresh Python traces.
[p,scenarios,root]=acc_inputs();
for h=[1 1.5 2]
    cfg=p; cfg.headway=h;
    for k=1:numel(scenarios)
        s=scenarios(k); x=acc_simulate(s,cfg);
        assert(all(isfinite(x(:,1:10)),'all'));
        assert(all(x(:,2:3)>=0,'all'));
        assert(all(x(:,6:7)<=2+1e-9,'all') && all(x(:,6:7)>=-4-1e-9,'all'));
        assert(all(x(x(:,9)>0,4)>0),'Collision: %s, headway %g',s.name,h);
    end
end
for k=1:numel(scenarios)
    s=scenarios(k); x=acc_simulate(s,p);
    reference=fullfile(root,'results','parity','traces',[s.name '.csv']);
    if ~isfile(reference), reference=fullfile(root,'results','traces',[s.name '.csv']); end
    py=readmatrix(reference);
    delta=abs(x(:,1:10)-py(:,1:10));
    assert(max(delta,[],'all')<1e-6,'Python/MATLAB parity: %s',s.name);
    assert(isequal(isinf(x(:,11)),isinf(py(:,11))));
    finite=isfinite(py(:,11));
    assert(all(abs(x(finite,11)-py(finite,11))<1e-4));
    fprintf('PASS Python/MATLAB parity: %s\n',s.name);
end
x=acc_simulate(scenarios(1),p);
assert(abs(x(end,2)-scenarios(1).set_speed)<0.15);
x=acc_simulate(scenarios(3),p); assert(abs(x(end,4)-x(end,5))<0.5);
x=acc_simulate(scenarios(4),p); assert(any(x(:,8)==2) && x(end,2)<0.1);
x=acc_simulate(scenarios(6),p); assert(all(x(x(:,1)>=25,8)==0));
s=scenarios(4); s.ego_speed=25; s.lead_speed=0; s.initial_gap=6; s.duration=8; s.events=[0 0];
x=acc_simulate(s,p); assert(any(x(:,4)<0),'Collision must not be hidden');
fprintf('PASS MATLAB behavior: 18 headway/scenario runs + edge cases\n');
end
