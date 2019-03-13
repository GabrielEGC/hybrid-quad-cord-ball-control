syms d I1 I2 l m m2 g ...
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
SY=solve(  [m*ddy==(u1+u2)*cos(th)-Ay-m*g,
        m2*ddy2==Ay-m2*g,
        ddy2==ddy+l*cos(al)*(dal)^2+l*sin(al)*ddal],[ddy,Ay,ddy2])

    
    
    %%
    solve([m*ddx==-(u1+u2)*sin(th)+Ax,,Ax==-m2*(ddx-l*sin(al)*(dal)^2+l*cos(al)*ddal)],{ddx,Ax})