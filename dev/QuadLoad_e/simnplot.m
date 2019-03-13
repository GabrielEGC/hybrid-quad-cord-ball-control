clear all, close all, clc
b=BnBPlant2D;
ytraj=b.simulate([0 3],[2; 0; 3; 0; 2.5; 0; 0; 0; -1]);
%
yHy1=ytraj.traj{1};
t = yHy1.pp.breaks;
y = yHy1.eval(t);
%
figure(1)
plot(t,y(2,:)), hold on
plot(t,y(4,:))
%
yHy2=ytraj.traj{2};
t = yHy2.pp.breaks;
y = yHy2.eval(t);
%
figure(1)
plot(t,y(2,:)), hold on
plot(t,y(4,:))