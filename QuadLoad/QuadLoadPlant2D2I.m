classdef QuadLoadPlant2D2I < HybridDrakeSystem
  properties
    l = 2;
    mq = 4;
    ml = 1;
    g = 9.81;
    I1 = 2;
    d = 0.25;
    
  end
  
  methods
    function obj = QuadLoadPlant2D2I()
      obj = obj@HybridDrakeSystem(...
        2, ...  % number of inputs
        13);     % number of outputs
      
      % create flight mode system
      sys = QuadLoadAsOnePhasePlant2D2I();
      obj = setInputFrame(obj,sys.getInputFrame);
      obj = setOutputFrame(obj,sys.getOutputFrame);
      
      sys2 = QuadLoadFlightPhasePlant2D2I();
      sys2 = setInputFrame(sys2,sys.getInputFrame);
      sys2 = setOutputFrame(sys2,sys.getOutputFrame);
      
      [obj,asone_mode] = addMode(obj,sys);  % add the 1st mode
      [obj,flight_mode] = addMode(obj,sys2);  % add the 2nd mode
      %obj = obj.setInputLimits(-300,300);
      obj = addTransition(obj, ...
        1, ...            % from mode
        @ForceGuard1, ...                    % q-r<=0 & qdot<=0 %andGuards(obj,g1,g2)
        @collisionDynamics12, ...    % transition method
        true, ...                 % not direct feedthrough %IMPORTANT
        true,2);%,...                   % time invariant
        %flight_mode);              % to mode 5%%%HUGE _ AVOID ERROR IN HyTO
      obj = addTransition(obj, ...
        2, ...            % from mode
        andGuards(obj,@LengthGuard,@IncrLengthGuard), ...                    % q-r<=0 & qdot<=0 %andGuards(obj,g1,g2)
        @collisionDynamics21, ...    % transition method
        false, ...                 % not direct feedthrough %IMPORTANT
        true,1);%,...                   % time invariant
        %asone_mode);              % to mode 
    obj = setSimulinkParam(obj,'InitialStep','1e-3','MaxStep','0.05');
    end

    
    function [g,dg] = ForceGuard1(obj,t,x,u)
      u1=u(1);u2=u(2);
      m=obj.mq;l=obj.l;
      al=x(6);dal=x(12);th=x(3);
        
      g = m*(l)^2*(dal)^2+cos(al-th)*(u1+u2);%  CONSIDER +EPS % CONSIDER g1.5=distance 
      g = g + eps;
      dg = [ 0, 0, 0, sin(al - th)*(u1 + u2), 0, 0, -sin(al - th)*(u1 + u2), 0, 0, 0, 0, 0, 2*dal*l^2*m, cos(al - th), cos(al - th)];
    end
    function [g,dg] = LengthGuard(obj,t,xv,u)
        x=xv(1);y=xv(2);x2=xv(4);y2=xv(5);
      g = obj.l-sqrt((x-x2)^2+(y-y2)^2);
      g = g + eps;
      dh22dx=[ -(x - x2)/((x - x2)^2 + (y - y2)^2)^(1/2), -(y - y2)/((x - x2)^2 + (y - y2)^2)^(1/2), 0, (x - x2)/((x - x2)^2 + (y - y2)^2)^(1/2), (y - y2)/((x - x2)^2 + (y - y2)^2)^(1/2), 0, 0, 0, 0, 0, 0, 0];
      dg = [0,dh22dx,0,0];
    end
    function [g,dg] = IncrLengthGuard(obj,t,xv,u)
        x=xv(1);y=xv(2);x2=xv(4);y2=xv(5);dx=xv(7);dy=xv(8);dx2=xv(10);dy2=xv(11);
        
      g = -((x-x2)*(dx-dx2)+(y-y2)*(dy-dy2));   % theta_st >= 0
      g = g + eps;
      dkdh22dx=[dx2 - dx, dy2 - dy, 0, dx - dx2, dy - dy2, 0, x2 - x, y2 - y, 0, x - x2, y - y2, 0];
      dg = [0,dkdh22dx,0,0];
    end

    function [xn,m,status,dxn] = collisionDynamics12(obj,m,t,x,u)%[xp,mode,status,dxp]
      l=obj.l;al=x(5);dal=x(10);
      xn = x;
      xn(4) = x(1)+l*sin(x(6));
      xn(5) = x(2)-l*cos(x(6));
      xn(6) = 0;
      xn(10) = x(7)+l*cos(x(6))*x(12);
      xn(11) = x(8)+l*sin(x(6))*x(12);
      xn(12) = 0;
      status = 0;                   
      m=2;
      dxndx =  [[ 1, 0, 0, 0, 0,              0, 0, 0, 0, 0, 0,         0]
                [ 0, 1, 0, 0, 0,              0, 0, 0, 0, 0, 0,         0]
                [ 0, 0, 1, 0, 0,              0, 0, 0, 0, 0, 0,         0]
                [ 1, 0, 0, 0, 0,      l*cos(al), 0, 0, 0, 0, 0,         0]
                [ 0, 1, 0, 0, 0,      l*sin(al), 0, 0, 0, 0, 0,         0]
                [ 0, 0, 0, 0, 0,              0, 0, 0, 0, 0, 0,         0]
                [ 0, 0, 0, 0, 0,              0, 1, 0, 0, 0, 0,         0]
                [ 0, 0, 0, 0, 0,              0, 0, 1, 0, 0, 0,         0]
                [ 0, 0, 0, 0, 0,              0, 0, 0, 1, 0, 0,         0]
                [ 0, 0, 0, 0, 0, -dal*l*sin(al), 1, 0, 0, 0, 0, l*cos(al)]
                [ 0, 0, 0, 0, 0,  dal*l*cos(al), 0, 1, 0, 0, 0, l*sin(al)]
                [ 0, 0, 0, 0, 0,              0, 0, 0, 0, 0, 0,         0]]; %8 states
      dxn = [zeros(12,2),dxndx,zeros(12,2)];
    end
    
    function [xn,m,status,dxn] = collisionDynamics21(obj,m,t,xv,u)%[xp,mode,status,dxp]
      x=xv(1);y=xv(2);th=xv(3);x2=xv(4);y2=xv(5);
      dx=xv(7);dy=xv(8);dth=xv(9);dx2=xv(10);dy2=xv(11);l=obj.l;
      aln = pi/2+atan2(y2-y,x2-x);
      xn=zeros(12,1);
        xn(1) = x;
        xn(2) = y;
        xn(3) = th;
        xn(4) = 0;
        xn(5) = 0;
        xn(6) = aln;
        xn(7) = dx;
        xn(8) = dy;
        xn(9) = dth;
        xn(10) = 0;
        xn(11) = 0;
        xn(12) = 1/l*((dx2-dx)*cos(aln)+(dy2-dy)*sin(aln));
      status = 0;
      m=1;
      dxndx = [[                                                                                                                                                     1,                                                                                                                                                      0, 0,                                                                                                                                                      0,                                                                                                                                                     0, 0,                                        0,                                       0, 0,                                       0,                                        0, 0]
[                                                                                                                                                     0,                                                                                                                                                      1, 0,                                                                                                                                                      0,                                                                                                                                                     0, 0,                                        0,                                       0, 0,                                       0,                                        0, 0]
[                                                                                                                                                     0,                                                                                                                                                      0, 1,                                                                                                                                                      0,                                                                                                                                                     0, 0,                                        0,                                       0, 0,                                       0,                                        0, 0]
[                                                                                                                                                     0,                                                                                                                                                      0, 0,                                                                                                                                                      0,                                                                                                                                                     0, 0,                                        0,                                       0, 0,                                       0,                                        0, 0]
[                                                                                                                                                     0,                                                                                                                                                      0, 0,                                                                                                                                                      0,                                                                                                                                                     0, 0,                                        0,                                       0, 0,                                       0,                                        0, 0]
[                                                                                                                   -(y - y2)/((x - x2)^2 + (y - y2)^2),                                                                                                                     (x - x2)/((x - x2)^2 + (y - y2)^2), 0,                                                                                                                     (y - y2)/((x - x2)^2 + (y - y2)^2),                                                                                                                   -(x - x2)/((x - x2)^2 + (y - y2)^2), 0,                                        0,                                       0, 0,                                       0,                                        0, 0]
[                                                                                                                                                     0,                                                                                                                                                      0, 0,                                                                                                                                                      0,                                                                                                                                                     0, 0,                                        1,                                       0, 0,                                       0,                                        0, 0]
[                                                                                                                                                     0,                                                                                                                                                      0, 0,                                                                                                                                                      0,                                                                                                                                                     0, 0,                                        0,                                       1, 0,                                       0,                                        0, 0]
[                                                                                                                                                     0,                                                                                                                                                      0, 0,                                                                                                                                                      0,                                                                                                                                                     0, 0,                                        0,                                       0, 1,                                       0,                                        0, 0]
[                                                                                                                                                     0,                                                                                                                                                      0, 0,                                                                                                                                                      0,                                                                                                                                                     0, 0,                                        0,                                       0, 0,                                       0,                                        0, 0]
[                                                                                                                                                     0,                                                                                                                                                      0, 0,                                                                                                                                                      0,                                                                                                                                                     0, 0,                                        0,                                       0, 0,                                       0,                                        0, 0]
[ ((y - y2)*(dx*x - dx*x2 - dx2*x + dx2*x2 + dy*y - dy*y2 - dy2*y + dy2*y2))/(l*abs(x - x2 + y*1i - y2*1i)*(x^2 - 2*x*x2 + x2^2 + y^2 - 2*y*y2 + y2^2)), -((x - x2)*(dx*x - dx*x2 - dx2*x + dx2*x2 + dy*y - dy*y2 - dy2*y + dy2*y2))/(l*abs(x - x2 + y*1i - y2*1i)*(x^2 - 2*x*x2 + x2^2 + y^2 - 2*y*y2 + y2^2)), 0, -((y - y2)*(dx*x - dx*x2 - dx2*x + dx2*x2 + dy*y - dy*y2 - dy2*y + dy2*y2))/(l*abs(x - x2 + y*1i - y2*1i)*(x^2 - 2*x*x2 + x2^2 + y^2 - 2*y*y2 + y2^2)), ((x - x2)*(dx*x - dx*x2 - dx2*x + dx2*x2 + dy*y - dy*y2 - dy2*y + dy2*y2))/(l*abs(x - x2 + y*1i - y2*1i)*(x^2 - 2*x*x2 + x2^2 + y^2 - 2*y*y2 + y2^2)), 0, -(y - y2)/(l*abs(x - x2 + y*1i - y2*1i)), (x - x2)/(l*abs(x - x2 + y*1i - y2*1i)), 0, (y - y2)/(l*abs(x - x2 + y*1i - y2*1i)), -(x - x2)/(l*abs(x - x2 + y*1i - y2*1i)), 0]]; %8 states
      dxn = [zeros(12,2),dxndx,zeros(12,2)];
    end
  end  
  methods (Static=true)
    function run
      b=BnBPlant2D;
      v=BallVisualizer2D(b);
      x=b.simulate([0 5],[2; 0; 3; 0; 2.5; 0; 0; 0; 1]);
      v.playback(x);
    end
  end
end
