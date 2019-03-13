 clear all, close all
syms d I1 I2 l m m2 g ...
    u1 u2
g=9.81;
m=4;
m2=1;
l=0.5;
I1=2;
d=0.25;

dt=0.01;
tf=2;
it=(1:tf/dt)*dt;

simp=1;

dx=0;
dy=0;
dth=0;
dal=6;

e=0.7;
x=0;
y=0;
th=0;
al=pi/4;
modeH=1;
vrest=0.5;%0.2
ResetTrigg=0;

for i=1:tf/dt
    u1=24.5;%+5*rand(1);
    u2=24.5;%+5*rand(1);
    switch(modeH)
        case 1
            ddx=-(2*m*u1*sin(th) + 2*m*u2*sin(th) + (3*m2*u1*sin(th))/2 + (3*m2*u2*sin(th))/2 - (m2*u1*sin(2*al - th))/2 - (m2*u2*sin(2*al - th))/2 - dal^2*l*m2^2*sin(al) - 2*dal^2*l*m*m2*sin(al))/((m + m2)*(2*m + m2));
            ddy=-(2*g*m^2 + g*m2^2 - 2*m*u1*cos(th) - 2*m*u2*cos(th) - (3*m2*u1*cos(th))/2 - (3*m2*u2*cos(th))/2 + (m2*u1*cos(2*al - th))/2 + (m2*u2*cos(2*al - th))/2 + 3*g*m*m2 + dal^2*l*m2^2*cos(al) + 2*dal^2*l*m*m2*cos(al))/((m + m2)*(2*m + m2));
            ddth=d/I1*(u1-u2);
            ddal=-(sin(al - th)*(u1 + u2))/(l*(2*m + m2));
            
            dx=dx+dt*ddx;
            dy=dy+dt*ddy;
            dth=dth+dt*ddth;
            dal=dal+dt*ddal;
            
            x=x+dt*dx;
            y=y+dt*dy;
            th=th+dt*dth;
            al=al+dt*dal;
            
            x2=x+l*sin(al);
            y2=y-l*cos(al);
            
            dx2=dx+l*cos(al)*dal;
            dy2=dy+l*sin(al)*dal;
            
            Flong(:,i)=(m2*(l*m*dal^2 + u1*cos(al - th) + u2*cos(al - th)))/(m + m2);
            L(:,i)=l;
            
            if Flong(:,i)<0
                modeH=2;
            end
            
            X(:,i)=[x;y;th;x2;y2;al];
            Xd(:,i)=[dx;dy;dth;dx2;dy2;dal];
            ModeH(:,i)=modeH;
            
        case 2
            ddx=-(u1+u2)*sin(th)/m;
            ddy=((u1+u2)*cos(th) - g*m)/m;
            ddth=d/I1*(u1-u2);
            ddx2=0;
            ddy2=-m2*g/m2;
            
            dx=dx+dt*ddx;
            dy=dy+dt*ddy;
            dth=dth+dt*ddth;
            dx2=dx2+dt*ddx2;
            dy2=dy2+dt*ddy2;
            
            x=x+dt*dx;
            y=y+dt*dy;
            th=th+dt*dth;
            x2=x2+dt*dx2;
            y2=y2+dt*dy2;
            
            X(:,i)=[x;y;th;x2;y2;al];
            Xd(:,i)=[dx;dy;dth;dx2;dy2;dal];
            
            L(:,i)=sqrt((x-x2)^2+(y-y2)^2);
            Flong(:,i)=0;
            al=pi/2+atan2(y2-y,x2-x);%Imaginary angle
            Vrel(:,i)=(dx2-dx)*sin(al)-(dy2-dy)*cos(al);
            Dh(:,i)=((x-x2)*(dx-dx2)+(y-y2)*(dy-dy2))/l;
            %Assuming initial negativity (Guard)
            if (L(:,i)>l && ((x-x2)*(dx-dx2)+(y-y2)*(dy-dy2))>0) %Impact
                % && ((x-x2)*(dx-dx2)+(y-y2)*(dy-dy2))>0
                al=pi/2+atan2(y2-y,x2-x);
                X(:,i)=[x;y;th;x2;y2;al];
                Xd(:,i)=[dx;dy;dth;dx2;dy2;dal];

                %x2=x+l*sin(al); %correction
                %y2=y-l*cos(al); %...
                %%&& ((x-x2)*(dx-dx2)+(y-y2)*(dy-dy2))>0
                if (dx2-dx)*sin(al)-(dy2-dy)*cos(al)<vrest || ResetTrigg%%|| (modeH_ant==2 && ((x-x2)*(dx-dx2)+(y-y2)*(dy-dy2))<0)
                    modeH=1;
                    dal=((dx2-dx)*cos(al)+(dy2-dy)*sin(al))/l;
                else
                    ResetTrigg=1;
                    modeH=2;
                    v2i=dx2*sin(al)-dy2*cos(al);
                    v1i=dx*sin(al)-dy*cos(al);
                    v2f=((m2*v2i+m*v1i)-m*e*(v2i-v1i))/(m+m2);
                    v1f=v2f+e*(v2i-v1i);
                    
                    v2ort=dx2*cos(al)+dy2*sin(al);
                    v1ort=dx*cos(al)+dy*sin(al);
                    
                    dx2=v2ort*cos(al)+v2f*sin(al);
                    dy2=v2ort*sin(al)-v2f*cos(al);
                    dx=v1ort*cos(al)+v1f*sin(al);
                    dy=v1ort*sin(al)-v1f*cos(al);
                    
                    x=x+dt*dx;
                    y=y+dt*dy;
                    
                    th=th+dt*dth;
                    x2=x2+dt*dx2;
                    y2=y2+dt*dy2;
                    disp('choque')
                    disp(i)
                end
            else
                ResetTrigg=0;
            end
            
            X(:,i)=[x;y;th;x2;y2;al];
            Xd(:,i)=[dx;dy;dth;dx2;dy2;dal];
            ModeH(:,i)=modeH;
    end
modeH_ant=modeH;
end

%%
figure(1)
Tdis=5*dt;
if simp
for i=0:(round(tf/Tdis)-1)
hold off
plot(X(1,round(1+i*Tdis/dt)),X(2,round(1+i*Tdis/dt)),'*r');

hold on
plot(X(1,:),X(2,:),'r')
plot(X(4,:),X(5,:),'b')

line(X(1,round(1+i*Tdis/dt))+d*cos(X(3,round(1+i*Tdis/dt)))*[-1 1],X(2,round(1+i*Tdis/dt))+d*sin(X(3,round(1+i*Tdis/dt)))*[-1 1],'linewidth',2)
plot(X(4,round(1+i*Tdis/dt)),X(5,round(1+i*Tdis/dt)),'or');
plot([X(1,round(1+i*Tdis/dt)) X(4,round(1+i*Tdis/dt))],[X(2,round(1+i*Tdis/dt)) X(5,round(1+i*Tdis/dt))]);
%plot(X(1,round(1+i*Tdis/dt))+l*sin(X(4,round(1+i*Tdis/dt))),X(2,round(1+i*Tdis/dt))-l*cos(X(4,round(1+i*Tdis/dt))),'or');

quiver(X(1,round(1+i*Tdis/dt))+d*cos(X(3,round(1+i*Tdis/dt))),X(2,round(1+i*Tdis/dt))+d*sin(X(3,round(1+i*Tdis/dt))),...
    -sin(X(3,round(1+i*Tdis/dt)))*u1/50,cos(X(3,round(1+i*Tdis/dt)))*u1/50,'g','MaxHeadSize',0.4)
quiver(X(1,round(1+i*Tdis/dt))-d*cos(X(3,round(1+i*Tdis/dt))),X(2,round(1+i*Tdis/dt))-d*sin(X(3,round(1+i*Tdis/dt))),...
    -sin(X(3,round(1+i*Tdis/dt)))*u2/50,cos(X(3,round(1+i*Tdis/dt)))*u2/50,'g','MaxHeadSize',0.4)

%DX=0.5*XRd(1,round(1+i*Tdis/T));DY=0.5*XRd(2,round(1+i*Tdis/T));
%quiver(XR(1,round(1+i*Tdis/T)),XR(2,round(1+i*Tdis/T)),DX,DY,'g','MaxHeadSize',0.4)
%quiver(0,0,XR(1,round(1+i*Tdis/T))*U(round(1+i*Tdis/T)),XR(2,round(1+i*Tdis/T))*U(round(1+i*Tdis/T)),'linewidth',2,'MaxHeadSize',0.4)
%legend('CoM', 'Trayectoria', '"Pierna" virtual', ...
% 'Vector velocidad CoM','Fuerza de Reacción')
axis([-2 2 -2 2])
title('x vs y')
xlabel('x (m)'),ylabel('y (m)'),
drawnow
%pause(Tdis/2)
%pause
%disp(i+1)
end
end
%%
figure(2)
plot(X(1,:)), hold on
plot(X(2,:))
plot(X(3,:))
plot(X(4,:))
plot(X(5,:))
xlabel('t (s)')
legend('Posición en x (m)','Posición en z (m)')
grid on
%%
figure(3)
plot(Xd(1,:)), hold on
plot(Xd(2,:))
plot(Xd(3,:))
plot(Xd(4,:))
plot(Xd(5,:))
xlabel('t (s)')
legend('dx(m/s)','dy(m/s)','dth(rad/s)','dx2(m/s)','dy2(m/s)')
grid on
%%
figure(4)
subplot(311)
plot(Flong(1,:)), hold on
legend('Parallel Force (m)')
subplot(312)
plot(L(1,:))
legend('Length(m)')
subplot(313)
plot(Vrel(1,:),'x'),axis([0 tf/dt -5 5])
legend('Vrel (m)')
grid on