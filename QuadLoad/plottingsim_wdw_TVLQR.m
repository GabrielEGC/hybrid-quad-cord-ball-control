ytraj=traj_testf;
%%
close all;
l=p.l;
d=p.d;
figure(3)
xs=1.5:0.01:2.5;
ys=0.5*sqrt(((xs-2)/0.2).^4+1);
ys2=-0.5*sqrt(((xs-2)/0.2).^4+1);
plot(xs,ys,'b'), hold on
plot(xs,ys2,'b')
nt=100;
for i=1:1
    yHy=ytraj;
    t = linspace(yHy.tspan(1),yHy.tspan(end),nt);%yHy.breaks;%linspace(yHy.tspan(1),yHy.tspan(end),50);%
    y = yHy.eval(t);
    %
    figure(3)
    v2pl=find(round(y(1,:))==2);
    v1pl=find(round(y(1,:))==1);
    
    x2forplot(v2pl)=y(5,v2pl);
    y2forplot(v2pl)=y(6,v2pl);
    x2forplot(v1pl)=y(2,v1pl)+l*sin(y(7,v1pl));
    y2forplot(v1pl)=y(3,v1pl)-l*cos(y(7,v1pl));
    plot(x2forplot,y2forplot)
    
    for k=1:size(y,2)
	clf
    plot(y(2,:),y(3,:)), hold on    
    plot(x2forplot,y2forplot)
    plot(xs,ys,'b'), hold on
    plot(xs,ys2,'b')
    if y(1,k)==2
        plot(y(5,k),y(6,k))
        line(y(2,k)+d*cos(y(4,k))*[-1 1],y(3,k)+d*sin(y(4,k))*[-1 1],'linewidth',2);
        plot([y(2,k) y(5,k)],[y(3,k) y(6,k)])
    else
        line(y(2,k)+d*cos(y(4,k))*[-1 1],y(3,k)+d*sin(y(4,k))*[-1 1],'linewidth',2);
        plot([y(2,k) y(2,k)+l*sin(y(7,k))],[y(3,k) y(3,k)-l*cos(y(7,k))])
    end
    pause(yHy.tspan(end)/nt)
    end
end
%
    figure(7); clf;
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