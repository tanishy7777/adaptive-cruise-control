function m = acc_metrics(x,s,p)
%ACC_METRICS Settling requires remaining in a +/-0.5 m/s band for >=5 s.
t=x(:,1); v=x(:,2); gap=x(:,4); desired=x(:,5); a=x(:,7); mode=x(:,8);
active=x(:,9)>0; follow=active & mode==1;
m.scenario=string(s.name); m.headway_s=p.headway;
m.min_gap_m=NaN; m.min_gap_margin_m=NaN; m.follow_gap_rmse_m=NaN;
if any(active)
    m.min_gap_m=min(gap(active)); m.min_gap_margin_m=min(gap(active)-desired(active));
end
if any(follow)
    m.follow_gap_rmse_m=sqrt(mean((gap(follow)-desired(follow)).^2));
end
m.below_desired_gap_s=sum(active(1:end-1) & gap(1:end-1)<desired(1:end-1))*p.dt;
post=t>=s.settle_after-1e-9;
bad=find(post & abs(v-s.final_speed)>0.5,1,'last');
if isempty(bad), idx=find(post,1); else, idx=bad+1; end
m.speed_settling_s=NaN;
if ~isempty(idx) && idx<=numel(t) && t(end)-t(idx)>=5
    m.speed_settling_s=t(idx)-s.settle_after;
end
m.set_speed_overshoot_kmh=max(0,max(v)-s.set_speed)*3.6;
m.peak_acceleration_mps2=max(a); m.peak_braking_mps2=-min(a);
jerk=diff(a)/p.dt;
m.rms_jerk_mps3=sqrt(mean(jerk.^2)); m.peak_jerk_mps3=max(abs(jerk));
m.emergency_s=sum(mode(1:end-1)==2)*p.dt;
m.mode_transitions=sum(diff(mode)~=0);
collision=find(active & gap<=0,1);
m.collision=~isempty(collision); m.first_collision_s=NaN;
if ~isempty(collision), m.first_collision_s=t(collision); end
m.final_speed_kmh=v(end)*3.6;
end
