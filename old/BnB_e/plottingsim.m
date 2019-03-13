clear all, close all, clc
[p,utraj,xtraj,z,traj_opt,F,info]=runDircolBnB_e;
info
%%
ytraj=xtraj;
for i=1:length(ytraj.traj)
    yHy=ytraj.traj{i};
    t = linspace(yHy.tspan(1),yHy.tspan(end),200);
    y = yHy.eval(t);
    %
    figure(1)
    plot(t,y(1,:)), hold on
    plot(t,y(3,:)), hold on
    pause
    plot(t,y(5,:)), hold on
    pause
    %
    figure(2)
    plot(t,y(7,:)), hold on
    pause
    plot(t,y(9,:)), hold on
    pause
    figure(4)
    plot(t,y(7,:)-y(9,:)), hold on
    plot(t,y(3,:)-y(5,:)), hold on
    plot(t,ones(size(t))*2), hold on
    plot(t,zeros(size(t))), hold on
end
%%
   figure(3); clf;
   fnplt(utraj);
%%

 %%
% ytraj=xtraj;
% for i=1:length(ytraj.traj)
%     yHy=ytraj.traj{i};
%     t = linspace(yHy.tspan(1),yHy.tspan(end),100);
%     y = yHy.eval(t);
%     %
%     figure(1)
%     plot(t,y(1,:)), hold on
%     plot(t,y(2,:)), hold on
%     pause
%     plot(t,y(3,:)), hold on
%     pause
%     %
%     figure(2)
%     plot(t,y(4,:)), hold on
%     pause
%     plot(t,y(5,:)), hold on
%     pause
% end