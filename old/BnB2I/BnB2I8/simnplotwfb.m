clear all, close all, clc
%%[xtraj]=BnBController.run();
plant = BnBPlant2D2I();
controller = BnBController2I(plant);
%
%v = plant.constructVisualizer;
sys_closedloop = feedback(plant,controller);
%
load('UT.mat'); 
%ytraj=sys_closedloop.simulate([0 utraj.tspan(end)],[1;  0; 0; 0; 0; 0;    0; 0; 0; 0; 0]);
ytraj=sys_closedloop.simulate([0 10],[1;  2; 0; 0; 0; pi/2;   0; 0; 0; 0; 3]);
%ytraj=sys_closedloop.simulate([0 6],[2;  0; 3; 0; 2.5; 0;   0; 0.6; 0; -1; 0]);
%
l=plant.l;
%%
figure(3)
xs=1.5:0.01:2.5;
ys=0.5*sqrt(((xs-2)/0.2).^4+1);
ys2=-0.5*sqrt(((xs-2)/0.2).^4+1);
plot(xs,ys,'b')
plot(xs,ys2,'b'), hold on
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
    if y(1,:)==2
        plot(y(4,:),y(5,:))
        for k=1:size(y,2)
            plot([y(2,k) y(4,k)],[y(3,k) y(5,k)])
            pause
        end
    else
        plot(y(2,:)+l*sin(y(6,:)),y(3,:)-l*cos(y(6,:))),
        for k=1:size(y,2)
            plot([y(2,k) y(2,k)+l*sin(y(6,k))],[y(3,k) y(3,k)-l*cos(y(6,k))])
            pause
        end
    end
    
    figure(4)
    plot(t,y(1,:)), hold on
    plot(t,y(6,:))
    plot(t,y(11,:))
    legend('m','al','dal')
end


