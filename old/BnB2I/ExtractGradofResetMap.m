%%Extracting grading of a reset map... SS
syms ml mq v2i v1i e 
v2f=((ml*v2i+mq*v1i)-mq*e*(v2i-v1i))/(mq+ml);
v1f=v2f+e*(v2i-v1i);
jacobian([v2f;v1f],[v1i;v2i])%Under refinement
%%
syms d I1 I2 l m m2 g t ...
    u1 u2 ...
    x dx ddx y dy ddy al dal ddal th dth ddth ...
    x2 dx2 ddx2 y2 dy2 ddy2 ...
    Ax Ay 
%m*ddy=(u1+u2)*cos(th)
%
S=solve(  [m*ddx==u2+Ax,
        m2*ddx2==-Ax,
        ddx2==ddx-l*sin(al)*(dal)^2+l*cos(al)*ddal,      
        m*ddy==u1-Ay-m*g,
        m2*ddy2==Ay-m2*g,
        ddy2==ddy+l*cos(al)*(dal)^2+l*sin(al)*ddal,
        m2*l^2*ddal==l*(-Ay*sin(al)+Ax*cos(al))],[ddx,Ax,ddx2,ddy,Ay,ddy2,ddal]);%...k?
ddx=simplify(S.ddx);
ddy=simplify(S.ddy);
ddal=simplify(S.ddal);
   
jacobian([ddx;ddy;0;0;ddal],[t,x,y,x2,y2,al,dx,dy,dx2,dy2,dal,u1,u2])%Under refinement


%%
%Reset collisionDynamics12
syms d I1 I2 l m m2 g t ...
    u1 u2 ...
    x dx ddx y dy ddy al dal ddal th dth ddth ...
    x2 dx2 ddx2 y2 dy2 ddy2 ...
    Ax Ay

xn(1) = x;
xn(2) = y;
xn(3) = x+l*sin(al);
xn(4) = y-l*cos(al);
xn(5) = 0;
xn(6) = dx;
xn(7) = dy;
xn(8) = dx+l*cos(al)*dal;
xn(9) = dy+l*sin(al)*dal;
xn(10) = 0;
jacobian(xn,[x,y,x2,y2,al,dx,dy,dx2,dy2,dal])%Under refinement
%%
h22=l-sqrt((x-x2)^2+(y-y2)^2);
simplify(jacobian(h22,[x,y,x2,y2,al,dx,dy,dx2,dy2,dal]));%Under refinement
kdh22=-((x-x2)*(dx-dx2)+(y-y2)*(dy-dy2));
simplify(jacobian(kdh22,[x,y,x2,y2,al,dx,dy,dx2,dy2,dal]))
%%
%Reset collisionDynamics21
syms d I1 I2 l m m2 g t ...
    u1 u2 ...
    x dx ddx y dy ddy al dal ddal th dth ddth ...
    x2 dx2 ddx2 y2 dy2 ddy2 ...
    Ax Ay
aln = pi/2+atan2(y2-y,x2-x);
xn(1) = x;
xn(2) = y;
xn(3) = 0;
xn(4) = 0;
xn(5) = aln;
xn(6) = dx;
xn(7) = dy;
xn(8) = 0;
xn(9) = 0;
xn(10) = 1/l*((dx2-dx)*cos(aln)+(dy2-dy)*sin(aln));
simplify(jacobian(xn,[x,y,x2,y2,al,dx,dy,dx2,dy2,dal]))

%%
x = sym('x','real');
x2 = sym('x2','real');
y = sym('y','real');
y2 = sym('y2','real');

