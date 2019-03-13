classdef BnBPlant2D2I < HybridDrakeSystem
  properties
    r = 0.5;  % radius of the ball
    cor = .4;  % coefficient of restitution
    l = 2;
    mq = 4;
    ml = 2;
    g=9.81;
  end
  
  methods
    function obj = BnBPlant2D2I()
      obj = obj@HybridDrakeSystem(...
        2, ...  % number of inputs
        9);     % number of outputs
      
      % create flight mode system
      sys = BallAsOnePhasePlant2D2I();
      obj = setInputFrame(obj,sys.getInputFrame);
      obj = setOutputFrame(obj,sys.getOutputFrame);
      
      sys2 = BallFlightPhasePlant2D2I();
      sys2 = setInputFrame(sys2,sys.getInputFrame);
      sys2 = setOutputFrame(sys2,sys.getOutputFrame);
      
      [obj,asone_mode] = addMode(obj,sys);  % add the 1st mode
      [obj,flight_mode] = addMode(obj,sys2);  % add the 2nd mode
      %obj = obj.setInputLimits(-300,300);
      obj = addTransition(obj, ...
        1, ...            % from mode
        @ForceGuard1, ...                    % q-r<=0 & qdot<=0 %andGuards(obj,g1,g2)
        @collisionDynamics12, ...    % transition method
        false, ...                 % not direct feedthrough %IMPORTANT
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

    
    function [g,dg] = ForceGuard1(obj,t,x,u) %%NEED DG
        al = pi/2+atan2(x(4)-x(2),x(3)-x(1));
        dal = -(x(4)-x(2))/((x(4)-x(2))^2 + (x(3)-x(1))^2)*(x(7)-x(5))+(x(3)-x(1))/((x(4)-x(2))^2 + (x(3)-x(1))^2)*(x(8)-x(6));
        
      g = obj.mq*(obj.l)^2*(dal)^2+u(1)*cos(al)-u(2)*sin(al);%  CONSIDER +EPS % CONSIDER g1.5=distance 
      g = g + eps;
      dg = [0,0,0,0,0,-u(1)*sin(al)-u(2)*cos(al),0,0,0,0,2*obj.mq*(obj.l)^2*dal,cos(x(5)),-sin(x(5))];
    end
    function [g,dg] = LengthGuard(obj,t,xv,u)%%NEED DG
        x=xv(1);y=xv(2);x2=xv(3);y2=xv(4);
      g = 0.05+obj.l-sqrt((x-x2)^2+(y-y2)^2);
      g = g + eps;
      dh22dx=[ -(x - x2)/((x - x2)^2 + (y - y2)^2)^(1/2), -(y - y2)/((x - x2)^2 + (y - y2)^2)^(1/2), (x - x2)/((x - x2)^2 + (y - y2)^2)^(1/2), (y - y2)/((x - x2)^2 + (y - y2)^2)^(1/2), 0, 0, 0, 0, 0, 0];
      dg = [0,dh22dx,0,0];
    end
    function [g,dg] = IncrLengthGuard(obj,t,xv,u)%%NEED DG
        x=xv(1);y=xv(2);x2=xv(3);y2=xv(4);dx=xv(5);dy=xv(6);dx2=xv(7);dy2=xv(8);
      g = -((x-x2)*(dx-dx2)+(y-y2)*(dy-dy2));   % theta_st >= 0
      g = g + eps;
      dkdh22dx=[ dx2 - dx, dy2 - dy, dx - dx2, dy - dy2, 0, x2 - x, y2 - y, x - x2, y - y2, 0];
      dg = [0,dkdh22dx,0,0];
    end

    function [xn,m,status,dxn] = collisionDynamics12(obj,m,t,x,u)%[xp,mode,status,dxp]
      l=obj.l;
      
      al = pi/2+atan2(x(4)-x(2),x(3)-x(1));
      dal = -(x(4)-x(2))/((x(4)-x(2))^2 + (x(3)-x(1))^2)*(x(7)-x(5))+(x(3)-x(1))/((x(4)-x(2))^2 + (x(3)-x(1))^2)*(x(8)-x(6));
        
      xn = x;
      status = 0;                   
      m=2;
      dxndx = eye(8)
      dxn = [zeros(8,2),dxndx,zeros(8,2)];
    end
    
    function [xn,m,status,dxn] = collisionDynamics21(obj,m,t,xv,u)%[xp,mode,status,dxp]
      x=xv(1);y=xv(2);x2=xv(3);y2=xv(4);dx=xv(5);dy=xv(6);dx2=xv(7);dy2=xv(8);l=obj.l;
      aln = pi/2+atan2(y2-y,x2-x);
      daln = 1/l*((dx2-dx)*cos(aln)+(dy2-dy)*sin(aln));
      xn=zeros(10,1);
        xn(1) = xv(1);
        xn(2) = xv(2);
        xn(3) = xv(3);
        xn(4) = xv(4);
        xn(5) = xv(5);
        xn(6) = xv(6);
        xn(7) = xv(5)+l*cos(aln)*daln;
        xn(8) = xv(6)+l*sin(aln)*daln;
        %xn(10) = ; %%%RECONSIDER IN ORIGINAL
      status = 0;
      m=1;
      dxndx = [[                                                                                                                                                                         1,                                                                                                                                                                          0,                                                                                                                                                                          0,                                                                                                                                                                         0, 0,                                    0,                                    0,                                   0,                                   0, 0]
[                                                                                                                                                                         0,                                                                                                                                                                          1,                                                                                                                                                                          0,                                                                                                                                                                         0, 0,                                    0,                                    0,                                   0,                                   0, 0]
[                                                                                                                                                                         0,                                                                                                                                                                          0,                                                                                                                                                                          0,                                                                                                                                                                         0, 0,                                    0,                                    0,                                   0,                                   0, 0]
[                                                                                                                                                                         0,                                                                                                                                                                          0,                                                                                                                                                                          0,                                                                                                                                                                         0, 0,                                    0,                                    0,                                   0,                                   0, 0]
[                                                                                                                                       -(y - y2)/((x - x2)^2 + (y - y2)^2),                                                                                                                                         (x - x2)/((x - x2)^2 + (y - y2)^2),                                                                                                                                         (y - y2)/((x - x2)^2 + (y - y2)^2),                                                                                                                                       -(x - x2)/((x - x2)^2 + (y - y2)^2), 0,                                    0,                                    0,                                   0,                                   0, 0]
[                                                                                                                                                                         0,                                                                                                                                                                          0,                                                                                                                                                                          0,                                                                                                                                                                         0, 0,                                    1,                                    0,                                   0,                                   0, 0]
[                                                                                                                                                                         0,                                                                                                                                                                          0,                                                                                                                                                                          0,                                                                                                                                                                         0, 0,                                    0,                                    1,                                   0,                                   0, 0]
[                                                                                                                                                                         0,                                                                                                                                                                          0,                                                                                                                                                                          0,                                                                                                                                                                         0, 0,                                    0,                                    0,                                   0,                                   0, 0]
[                                                                                                                                                                         0,                                                                                                                                                                          0,                                                                                                                                                                          0,                                                                                                                                                                         0, 0,                                    0,                                    0,                                   0,                                   0, 0]
[ ((cos(pi/2 + atan2(y2 - y, x2 - x))*(dy - dy2)*(y - y2))/((x - x2)^2 + (y - y2)^2) - (sin(pi/2 + atan2(y2 - y, x2 - x))*(dx - dx2)*(y - y2))/((x - x2)^2 + (y - y2)^2))/l, -((cos(pi/2 + atan2(y2 - y, x2 - x))*(dy - dy2)*(x - x2))/((x - x2)^2 + (y - y2)^2) - (sin(pi/2 + atan2(y2 - y, x2 - x))*(dx - dx2)*(x - x2))/((x - x2)^2 + (y - y2)^2))/l, -((cos(pi/2 + atan2(y2 - y, x2 - x))*(dy - dy2)*(y - y2))/((x - x2)^2 + (y - y2)^2) - (sin(pi/2 + atan2(y2 - y, x2 - x))*(dx - dx2)*(y - y2))/((x - x2)^2 + (y - y2)^2))/l, ((cos(pi/2 + atan2(y2 - y, x2 - x))*(dy - dy2)*(x - x2))/((x - x2)^2 + (y - y2)^2) - (sin(pi/2 + atan2(y2 - y, x2 - x))*(dx - dx2)*(x - x2))/((x - x2)^2 + (y - y2)^2))/l, 0, -cos(pi/2 + atan2(y2 - y, x2 - x))/l, -sin(pi/2 + atan2(y2 - y, x2 - x))/l, cos(pi/2 + atan2(y2 - y, x2 - x))/l, sin(pi/2 + atan2(y2 - y, x2 - x))/l, 0]]; %8 states
      dxn = [zeros(10,2),dxndx,zeros(10,2)];
            
      dxndx = eye(8)
      dxn = [zeros(8,2),dxndx,zeros(8,2)];
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
