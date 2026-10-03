function run_simulink()
%RUN_SIMULINK Simulate all baseline cases and assert MATLAB/Simulink parity.
[p,scenarios,root]=acc_inputs();
model=build_simulink_model(p);
out=fullfile(root,'results','matlab');
if ~isfolder(out), mkdir(out); end
cleanup=onCleanup(@() close_system(model,0)); %#ok<NASGU>
for k=1:numel(scenarios)
    s=scenarios(k);
    in=Simulink.SimulationInput(model);
    in=in.setVariable('vehicle_initial',[s.ego_speed;s.lead_speed;s.initial_gap;0],'Workspace',model);
    in=in.setVariable('memory_initial',[0;0;s.set_speed-s.ego_speed;0],'Workspace',model);
    in=in.setVariable('scenario_input',acc_scenario_signal(s,p),'Workspace',model);
    in=in.setModelParameter('StopTime',num2str(s.duration));
    result=sim(in);
    log=result.get('trace_log');
    x=[log.time,log.signals.values];
    expected=acc_simulate(s,p);
    assert(isequal(size(x),size(expected)),'Unexpected Simulink trace shape');
    delta=abs(x(:,1:10)-expected(:,1:10));
    assert(max(delta,[],'all')<1e-7,'MATLAB/Simulink mismatch for %s',s.name);
    assert(isequal(isinf(x(:,11)),isinf(expected(:,11))),'TTC mask mismatch');
    finite=isfinite(expected(:,11));
    assert(all(abs(x(finite,11)-expected(finite,11))<1e-6),'TTC mismatch');
    writematrix(x,fullfile(out,[s.name '_simulink.csv']));
    fprintf('PASS Simulink parity: %s (max error %.3g)\n',s.name,max(delta,[],'all'));
end
try
    print(['-s' model],'-dpng',fullfile(root,'models','acc_aeb.png'));
catch err
    warning('acc:diagram','Model passed; diagram export unavailable: %s',err.message);
end
end
