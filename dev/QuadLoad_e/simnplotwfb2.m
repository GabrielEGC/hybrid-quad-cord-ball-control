clear all, close all, clc
%%[xtraj]=BnBController.run();
plant = QuadLoadPlant2D2I();
controller = QuadLoadController2I(plant);
%
%v = plant.constructVisualizer;
sys_closedloop = feedback(plant,controller);
%
%load('UT.mat'); 
%ytraj=sys_closedloop.simulate([0 utraj.tspan(end)],[2; 0; 3; 0; 2.5; 0; 0.6; 0; -1]);
ytraj=sys_closedloop.simulate([0 3],[2;   0; 3; 0; 0; 2.75; 0;    1; 0; 0; 0; 0; 0]);

%ytraj=sys_closedloop.simulate([0 6],[1;  2; 0; 0; 0; pi/2;   0; 0; 0; 0; 2]);


%%
l=plant.l;
for i=1:length(ytraj.traj)
    yHy=ytraj.traj{i};
    t = linspace(yHy.tspan(1),yHy.tspan(end),50);%yHy.pp.breaks;
    y = yHy.eval(t);
    %
    figure(1)
    plot(t,y(1,:)), hold on
    plot(t,y(2,:))
    if y(1,2)==2
        plot(t,y(5,:))
    else
        plot(t,y(2,:)+l*sin(y(7,:))),
    end
    
    legend('m','x','x2')
    
    figure(2)
    plot(t,y(1,:)), hold on
    plot(t,y(3,:))
    if y(1,2)==2
        plot(t,y(6,:))
    else
        plot(t,y(3,:)-l*cos(y(7,:))),
    end
    legend('m','y','y2')
    %
    figure(3)
    plot(y(2,:),y(3,:)), hold on
    if y(1,:)==2
        plot(y(5,:),y(6,:))
        for k=1:size(y,2)
            plot([y(2,k) y(5,k)],[y(3,k) y(6,k)])
            pause
        end
    else
        plot(y(2,:)+l*sin(y(7,:)),y(3,:)-l*cos(y(7,:))),
        for k=1:size(y,2)
            plot([y(2,k) y(2,k)+l*sin(y(7,k))],[y(3,k) y(3,k)-l*cos(y(7,k))])
            pause
        end
    end
    
    figure(4)
    plot(t,y(1,:)), hold on
    plot(t,y(7,:))
    plot(t,y(13,:))
    legend('m','al','dal')
    pause
end


