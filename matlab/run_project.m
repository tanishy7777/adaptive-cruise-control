function run_project()
%RUN_PROJECT Run baseline scenarios, plots, and parameter studies in MATLAB.
[p,scenarios,root]=acc_inputs();
out=fullfile(root,'results','matlab');
if ~isfolder(out), mkdir(out); end
names={'time','ego_speed','lead_speed','gap','desired_gap','command',...
    'acceleration','mode','lead_present','reference_speed','ttc'};
baseline=cell(numel(scenarios),1); headways={}; controllers=cell(3,1);
for k=1:numel(scenarios)
    s=scenarios(k); x=acc_simulate(s,p);
    baseline{k}=acc_metrics(x,s,p);
    writetable(array2table(x,'VariableNames',names),fullfile(out,[s.name '.csv']));
    f=figure('Visible','off','Position',[0 0 1100 900]);
    tiledlayout(4,1); t=x(:,1); lead=x(:,3)*3.6; gap=x(:,4);
    lead(x(:,9)==0)=NaN; gap(x(:,9)==0)=NaN;
    nexttile; plot(t,[x(:,2)*3.6,lead]); yline(s.set_speed*3.6,':');
    ylabel('Speed [km/h]'); legend('Ego','Lead','Set speed'); grid on;
    title(s.title,'Interpreter','none');
    nexttile; plot(t,[gap,x(:,5)]); ylabel('Gap [m]'); legend('Actual','Desired'); grid on;
    nexttile; plot(t,x(:,[6 7])); ylabel('Acceleration [m/s^2]'); legend('Command','Actual'); grid on;
    nexttile; stairs(t,x(:,8)); yticks([0 1 2]); yticklabels({'CRUISE','FOLLOW','EMERGENCY'});
    ylim([-0.2 2.2]); xlabel('Time [s]'); grid on;
    exportgraphics(f,fullfile(out,[s.name '.png']),'Resolution',150); close(f);
end
writetable(struct2table(vertcat(baseline{:})),fullfile(out,'baseline_metrics.csv'));
for h=[1 1.5 2]
    cfg=p; cfg.headway=h;
    for k=1:numel(scenarios)
        s=scenarios(k);
        headways{end+1}=acc_metrics(acc_simulate(s,cfg),s,cfg); %#ok<AGROW>
    end
end
writetable(struct2table(vertcat(headways{:})),fullfile(out,'headway_metrics.csv'));
labels={'P','PI','PID'};
for k=1:3
    cfg=p;
    if k==1, cfg.ki=0; end
    if k<3, cfg.kd=0; end
    s=scenarios(3); m=acc_metrics(acc_simulate(s,cfg),s,cfg);
    m.controller=string(labels{k}); controllers{k}=m;
end
writetable(struct2table(vertcat(controllers{:})),fullfile(out,'controller_metrics.csv'));
disp(struct2table(vertcat(baseline{:})));
fprintf('Saved MATLAB results to %s\n',out);
end
