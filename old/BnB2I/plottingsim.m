clear all, close all, clc
[p,utraj,xtraj,z,traj_opt,F,info]=runDircolBnB2I;
info
ytraj=xtraj;
%%
l=p.l;
for i=1:length(ytraj.traj)
    yHy=ytraj.traj{i};
    t = linspace(yHy.tspan(1),yHy.tspan(end),100);
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
    pause
    
    figure(2)
    plot(t,y(1,:)), hold on
    plot(t,y(3,:))
    if y(1,2)==2
        plot(t,y(5,:))
    else
        plot(t,y(3,:)-l*cos(y(6,:))),
    end
    legend('m','y','y2')
    pause
    %
    figure(3)
    plot(y(2,:),y(3,:)), hold on
    if y(1,2)==2
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
    pause
end
%
    figure(5); clf;
    subplot(211)
    fnplt(utraj,1);hold on 
    subplot(212)
    fnplt(utraj,2);hold on
%%
% for i=1:length(utraj.traj)
%     uHy = utraj.traj{i};
%     t = linspace(uHy.tspan(1),uHy.tspan(end),100);
%     u = uHy.eval(t);
%     figure(5)
%     plot(t,y(1,:)), hold on
%     plot(t,y(3,:))
%     plot(t,y(5,:))
%     legend('m','y','y2')
% end
%
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