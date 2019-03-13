clear all, close all, clc
%%[xtraj]=BnBController.run();
plant = BnBPlant2D();
controller = BnBController(plant);
%v = plant.constructVisualizer;
sys_closedloop = feedback(plant,controller);
load('UT.mat'); 
ytraj=sys_closedloop.simulate([0 10],[2;   0; 3; 0; 2.5;    0; 0.6; 0; -1]);
%%
for i=1:length(ytraj.traj)
    yHy=ytraj.traj{i};
    t = yHy.pp.breaks;
    y = yHy.eval(t);
    %
    figure(1)
    plot(t,y(1,:)), hold on
    plot(t,y(3,:))
    plot(t,y(5,:))
    %
    figure(2)
    plot(t,y(7,:)), hold on
    pause
    plot(t,y(9,:))
    
end