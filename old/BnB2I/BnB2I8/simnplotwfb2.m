clear all, close all, clc
%%[xtraj]=BnBController.run();
plant = BnBPlant2D2I();
controller = BnBController2I(plant);
%
%v = plant.constructVisualizer;
sys_closedloop = feedback(plant,controller);
%
%load('UT.mat'); 
%ytraj=sys_closedloop.simulate([0 utraj.tspan(end)],[2; 0; 3; 0; 2.5; 0; 0.6; 0; -1]);
%ytraj=sys_closedloop.simulate([0 6],[2;  0; 3; 0; 2.5; 0;   0; 0.6; 0; -1; 0]);

ytraj=sys_closedloop.simulate([0 6],[2;  0; 0; 1; 0;   0; 0; 0; 8]);

%
l=plant.l;
%%
for i=1:length(ytraj.traj)
    yHy=ytraj.traj{i};
    t = yHy.pp.breaks;
    y = yHy.eval(t);
    %
    figure(1)
    plot(t,y(1,:)), hold on
    plot(t,y(2,:))
    plot(t,y(4,:))
    legend('m','x','x2')
    
    figure(2)
    plot(t,y(1,:)), hold on
    plot(t,y(3,:))
    plot(t,y(5,:))
    legend('m','y','y2')
    %
    figure(3)
    plot(y(2,:),y(3,:)), hold on
    plot(y(4,:),y(5,:))
    for k=1:size(y,2)
        plot([y(2,k) y(4,k)],[y(3,k) y(5,k)])
        pause
     end
%         
%     figure(4)
%     plot(t,y(1,:)), hold on
%     plot(t,y(6,:))
%     plot(t,y(11,:))
%     legend('m','al','dal')
    pause
end


