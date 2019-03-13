  clear all, close all, clc
%%[xtraj]=BnBController.run();
plant = BnBPlant2D2I();
controller = BnBController2I(plant);
%
%v = plant.constructVisualizer;
sys_closedloop = feedback(plant,controller);
%
load('UTPC.mat'); 
ytraj=sys_closedloop.simulate([0 utraj.tspan(end)],[1;  0; 0; 0; 0; 0;    0; 0; 0; 0; 0]);
%ytraj=sys_closedloop.simulate([0 6],[2;  0; 3; 0; 2.5; 0;   0; 0.6; 0; -1; 0]);
%
l=plant.l;
%%
figure(3)
xs=1.4:0.01:2.6;
ys=0.5*sqrt(((xs-2)/0.2).^4+1);
ys2=-0.5*sqrt(((xs-2)/0.2).^4+1);
plot(xs,ys,'b'), hold on
plot(xs,ys2,'b')
for i=1:length(ytraj.traj)
    yHy=ytraj.traj{i};
    t = linspace(yHy.tspan(1),yHy.tspan(end),50);%yHy.pp.breaks;
    y = yHy.eval(t);
    %
    figure(1)
    plot(t,y(1,:)), hold on
    plot(t,y(2,:))
    if y(1,2)==2
        plot(t,y(4,:))
    else
        plot(t,y(2,:)+l*sin(y(6,:))),
    end
    
    legend('m','x','x2')
    
    figure(2)
    plot(t,y(1,:)), hold on
    plot(t,y(3,:))
    if y(1,2)==2
        plot(t,y(5,:))
    else
        plot(t,y(3,:)-l*cos(y(6,:))),
    end
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
    %
    figure(4)
    plot(t,y(1,:)), hold on
    plot(t,y(6,:))
    plot(t,y(11,:))
    legend('m','al','dal')
end