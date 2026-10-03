function model = build_simulink_model(p)
%BUILD_SIMULINK_MODEL Generate a real, editable discrete Simulink block model.
% No Stateflow chart or specialty control/driving toolbox is required.
[defaults,scenarios,root]=acc_inputs();
if nargin==0, p=defaults; end
model='acc_aeb';
if bdIsLoaded(model), close_system(model,0); end
new_system(model);
set_param(model,'SolverType','Fixed-step','Solver','FixedStepDiscrete',...
    'FixedStep',num2str(p.dt,17),'StopTime',num2str(scenarios(3).duration),...
    'ReturnWorkspaceOutputs','on');
add_block('simulink/Sources/From Workspace',[model '/Scenario'],...
    'VariableName','scenario_input','Interpolate','off','OutputAfterFinalValue','Holding final value',...
    'SampleTime',num2str(p.dt,17),'Position',[25 55 160 95]);
add_block('simulink/Signal Routing/Demux',[model '/Inputs'],...
    'Outputs','3','Position',[195 35 200 145]);
add_block('simulink/Discrete/Unit Delay',[model '/Vehicle state'],...
    'InitialCondition','vehicle_initial','SampleTime',num2str(p.dt,17),...
    'Position',[260 290 365 345]);
add_block('simulink/Discrete/Unit Delay',[model '/Controller memory'],...
    'InitialCondition','memory_initial','SampleTime',num2str(p.dt,17),...
    'Position',[425 365 540 415]);
assignments="";
fields=fieldnames(p);
for k=1:numel(fields)
    assignments=assignments+sprintf('p.%s = %.17g;\n',fields{k},p.(fields{k}));
end
controllerCode=sprintf(['function [u,next,desired,reference,ttc] = f(vehicle,memory,present,setSpeed)\n' ...
    '%%#codegen\n%s' ...
    '[u,next,desired,reference,ttc] = acc_controller(vehicle(1),vehicle(2),vehicle(3),' ...
    'present,setSpeed,vehicle(4),memory,p);\nend\n'],assignments);
addFunction('Supervisor and PID',[290 40 480 205],controllerCode);
plantCode=sprintf(['function next = f(vehicle,u,leadAcceleration)\n%%#codegen\n' ...
    'next = acc_plant(vehicle,u,leadAcceleration,%.17g,%.17g);\nend\n'],p.dt,p.actuator_tau);
addFunction('Vehicle dynamics',[595 245 750 335],plantCode);
observeCode=sprintf(['function y = f(vehicle,memory,u,desired,reference,ttc,present)\n%%#codegen\n' ...
    'y = [vehicle(1);vehicle(2);vehicle(3);desired;u;vehicle(4);memory(1);present;reference;ttc];\nend\n']);
addFunction('Measurements',[650 30 790 200],observeCode);
add_block('simulink/Sinks/To Workspace',[model '/Trace'],...
    'VariableName','trace_log','SaveFormat','Structure With Time',...
    'MaxDataPoints','inf','Position',[865 55 965 90]);
add_block('simulink/Sinks/Scope',[model '/Scope'],...
    'NumInputPorts','4','Position',[1080 150 1160 265]);
scopeConfig=get_param([model '/Scope'],'ScopeConfiguration');
scopeConfig.LayoutDimensions=[4 1];
displayCode=sprintf(['function [speed_kmh,gap_m,accel_mps2,mode] = f(y)\n%%#codegen\n' ...
    'speed_kmh = y(1:2)*3.6; gap_m = y(3:4); accel_mps2 = y(5:6); mode = y(7);\nend\n']);
addFunction('Dashboard signals',[860 155 1000 270],displayCode);
wire('Scenario/1','Inputs/1');
wire('Inputs/1','Vehicle dynamics/3');
wire('Inputs/2','Supervisor and PID/3'); wire('Inputs/2','Measurements/7');
wire('Inputs/3','Supervisor and PID/4');
wire('Vehicle state/1','Supervisor and PID/1');
wire('Vehicle state/1','Vehicle dynamics/1'); wire('Vehicle state/1','Measurements/1');
wire('Controller memory/1','Supervisor and PID/2');
wire('Supervisor and PID/1','Vehicle dynamics/2'); wire('Supervisor and PID/1','Measurements/3');
wire('Supervisor and PID/2','Controller memory/1'); wire('Supervisor and PID/2','Measurements/2');
wire('Supervisor and PID/3','Measurements/4'); wire('Supervisor and PID/4','Measurements/5');
wire('Supervisor and PID/5','Measurements/6');
wire('Vehicle dynamics/1','Vehicle state/1');
wire('Measurements/1','Trace/1'); wire('Measurements/1','Dashboard signals/1');
for k=1:4
    wire(sprintf('Dashboard signals/%d',k),sprintf('Scope/%d',k));
end
s=scenarios(3); w=get_param(model,'ModelWorkspace');
assignin(w,'vehicle_initial',[s.ego_speed;s.lead_speed;s.initial_gap;0]);
assignin(w,'memory_initial',[0;0;s.set_speed-s.ego_speed;0]);
assignin(w,'scenario_input',acc_scenario_signal(s,p));
note=Simulink.Annotation(model,sprintf(['ACC + AEB | 50 Hz | SI units\n' ...
    'CRUISE / FOLLOW / EMERGENCY_BRAKE\n' ...
    'Explicit state delays break feedback loops. Run run_simulink for all scenarios.']));
note.Position=[25 460 650 515];
set_param(model,'SimulationCommand','update');
if ~isfolder(fullfile(root,'models')), mkdir(fullfile(root,'models')); end
save_system(model,fullfile(root,'models',[model '.slx']));
fprintf('Built %s\n',fullfile(root,'models',[model '.slx']));

    function addFunction(name,position,code)
        path=[model '/' name];
        add_block('simulink/User-Defined Functions/MATLAB Function',path,'Position',position);
        chart=find(sfroot,'-isa','Stateflow.EMChart','Path',path);
        chart.Script=char(code);
    end
    function wire(source,destination)
        add_line(model,source,destination,'autorouting','on');
    end
end
