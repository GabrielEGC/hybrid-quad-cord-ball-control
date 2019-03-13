%%Extracting grading of a reset map... SS
syms ml mq v2i v1i e 
v2f=((ml*v2i+mq*v1i)-mq*e*(v2i-v1i))/(mq+ml);
v1f=v2f+e*(v2i-v1i);
jacobian([v2f;v1f],[v1i;v2i])%Under refinement

%%

%%
syms d I1 I2 l m m2 g t...
    u1 u2 ...
    x dx ddx y dy ddy al dal ddal th dth ddth ...
    x2 dx2 ddx2 y2 dy2 ddy2 ...
    Ax Ay 
%m*ddy=(u1+u2)*cos(th)
%
S=solve(  [m*ddx==-(u1+u2)*sin(th)+Ax,
        m2*ddx2==-Ax,
        ddx2==ddx-l*sin(al)*(dal)^2+l*cos(al)*ddal,      
        m*ddy==(u1+u2)*cos(th)-Ay-m*g,
        m2*ddy2==Ay-m2*g,
        ddy2==ddy+l*cos(al)*(dal)^2+l*sin(al)*ddal,
        m2*l^2*ddal==l*(-Ay*sin(al)+Ax*cos(al))],[ddx,Ax,ddx2,ddy,Ay,ddy2,ddal]);

%%
ddx=simplify(S.ddx);
ddy=simplify(S.ddy);
ddth=d/I1*(u1-u2);
ddal=simplify(S.ddal);
jacobian([ddx;ddy;ddth;0;0;ddal],[t,x,y,th,x2,y2,al,dx,dy,dth,dx2,dy2,dal,u1,u2])%Under refinement
%%
syms d I1 I2 l m m2 g t...
    u1 u2 ...
    x dx ddx y dy ddy al dal ddal th dth ddth ...
    x2 dx2 ddx2 y2 dy2 ddy2 ...
    Ax Ay 
ddx=-(u1+u2)*sin(th)/m;
ddy=((u1+u2)*cos(th) - g*m)/m;
ddth=d/I1*(u1-u2);
ddx2=0;
ddy2=-m2*g/m2;

jacobian([ddx;ddy;ddth;ddx2;ddy2;0],[t,x,y,th,x2,y2,al,dx,dy,dth,dx2,dy2,dal,u1,u2])%Under refinement

%% ForceGuard1

h12=m*(l)^2*(dal)^2+cos(al-th)*(u1+u2);
jacobian(h12,[t,x,y,th,x2,y2,al,dx,dy,dth,dx2,dy2,dal,u1,u2])%Under refinement

%%
%Reset collisionDynamics12
syms d I1 I2 l m m2 g t ...
    u1 u2 ...
    x dx ddx y dy ddy al dal ddal th dth ddth ...
    x2 dx2 ddx2 y2 dy2 ddy2 ...
    Ax Ay

xn(1) = x;
xn(2) = y;
xn(3) = th;
xn(4) = x+l*sin(al);
xn(5) = y-l*cos(al);
xn(6) = 0;
xn(7) = dx;
xn(8) = dy;
xn(9) = dth;
xn(10) = dx+l*cos(al)*dal;
xn(11) = dy+l*sin(al)*dal;
xn(12) = 0;
jacobian(xn,[x,y,th,x2,y2,al,dx,dy,dth,dx2,dy2,dal])%Under refinement
%%
h22=l-sqrt((x-x2)^2+(y-y2)^2);
simplify(jacobian(h22,[x,y,th,x2,y2,al,dx,dy,dth,dx2,dy2,dal]))%Under refinement
kdh22=-((x-x2)*(dx-dx2)+(y-y2)*(dy-dy2));
simplify(jacobian(kdh22,[x,y,th,x2,y2,al,dx,dy,dth,dx2,dy2,dal]))
%%
%Reset collisionDynamics21
syms d I1 I2 l m m2 g t ...
    u1 u2 ...
    x dx ddx y dy ddy al dal ddal th dth ddth ...
    x2 dx2 ddx2 y2 dy2 ddy2 ...
    Ax Ay ml mq
x = sym('x','real');
x2 = sym('x2','real');
y = sym('y','real');
y2 = sym('y2','real');
aln = pi/2+atan2(y2-y,x2-x);
        v2i=dx2*sin(aln)-dy2*cos(aln);
        v1i=dx*sin(aln)-dy*cos(aln);
        v2f=(ml*v2i+mq*v1i);
        v1f=v2f;
        v2ort=dx2*cos(aln)+dy2*sin(aln);
        v1ort=dx*cos(aln)+dy*sin(aln);
        dxn=v1ort*cos(aln)+v1f*sin(aln);
        dyn=v1ort*sin(aln)-v1f*cos(aln);   
        dx2n=v2ort*cos(aln)+v2f*sin(aln);
        dy2n=v2ort*sin(caln)-v2f*cos(aln);
      
        xn(1) = x;
        xn(2) = y;
        xn(3) = th;
        xn(4) = 0;
        xn(5) = 0;
        xn(6) = aln;
        xn(7) = dxn;
        xn(8) = dyn;
        xn(9) = dth;
        xn(10) = 0;
        xn(11) = 0;
        xn(12) = 1/l*((dx2n-dxn)*cos(aln)+(dy2n-dyn)*sin(aln));
        
simplify(jacobian(xn,[x,y,th,x2,y2,al,dx,dy,dth,dx2,dy2,dal]))

%%
%Reset collisionDynamics22
syms d I1 I2 m ml g t ...
    u1 u2 ...
    x dx ddx y dy ddy al dal ddal th dth ddth ...
    x2 dx2 ddx2 y2 dy2 ddy2 ...
    Ax Ay ml mq v2i v1i e

x = sym('x','real');
x2 = sym('x2','real');
y = sym('y','real');
y2 = sym('y2','real');


aln = pi/2+atan2(y2-y,x2-x);
v2i=dx2*sin(aln)-dy2*cos(aln);
v1i=dx*sin(aln)-dy*cos(aln);
v2f=((ml*v2i+mq*v1i)-mq*e*(v2i-v1i))/(mq+ml);
v1f=v2f+e*(v2i-v1i);
v2ort=dx2*cos(aln)+dy2*sin(aln);
v1ort=dx*cos(aln)+dy*sin(aln);
dx2n=v2ort*cos(aln)+v2f*sin(aln);
dy2n=v2ort*sin(aln)-v2f*cos(aln);
dxn=v1ort*cos(aln)+v1f*sin(aln);
dyn=v1ort*sin(aln)-v1f*cos(aln);

xn(1) = x;
xn(2) = y;
xn(3) = th;
xn(4) = x2;
xn(5) = y2;
xn(6) = 0;
xn(7) = dxn;
xn(8) = dyn;
xn(9) = dth;
xn(10) = dx2n;
xn(11) = dy2n;
xn(12) = 0;
simplify(jacobian(xn,[x,y,th,x2,y2,al,dx,dy,dth,dx2,dy2,dal]))

%%
x = sym('x','real');
x2 = sym('x2','real');
y = sym('y','real');
y2 = sym('y2','real');

%%
syms d I1 I2 l m m2 g t ...
    u1 u2 ...
    x1 dx ddx y dy ddy al dal ddal th dth ddth ...
    x2 dx2 ddx2 y2 dy2 ddy2

    xs=x1-d*cos(th);
    ys=y-d*sin(th);
f=(ys/0.25)^2-((xs-2)/0.1)^4-1;
jacobian(f,[x1,y,th,x2,y2,al,dx,dy,dth,dx2,dy2,dal,u1,u2])



