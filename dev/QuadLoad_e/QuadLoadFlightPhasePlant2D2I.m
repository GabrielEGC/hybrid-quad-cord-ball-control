  classdef QuadLoadFlightPhasePlant2D2I < DrakeSystem
  properties
    l = 1;
    mq = 4;
    ml = 1;
    g = 9.81;
    I1 = 2;
    d = 0.25;
  end
  
  methods
    function obj = QuadLoadFlightPhasePlant2D2I()
      obj = obj@DrakeSystem(...
        12, ... % number of continuous states
        0, ... % number of discrete states
        2, ... % number of inputs
        13, ... % number of outputs
        false, ... % not direct feedthrough
        true); % time invariant
    obj=obj.setInputLimits(100*[-1;-1],100*[1;1]);
    end
    
    function [f,df] = dynamics(obj,t,x,u)
      u1=u(1);u2=u(2);
      m=obj.mq;g=obj.g;m2=obj.ml;d=obj.d;I1=obj.I1;
      l=obj.l;th=x(3);
        
      ddx=-(u1+u2)*sin(th)/m;
      ddy=((u1+u2)*cos(th) - g*m)/m;
      ddth=d/I1*(u1-u2);
      ddx2=0;
      ddy2=-m2*g/m2;
        
      f = [x(7:12);ddx;ddy;ddth;ddx2;ddy2;0];  % qddot = [0; -g]; x=[q,qdot]
      df = sparse(12,15);
      df(1:6,8:13) = eye(6);
      df(7:12,:) =  [[ 0, 0, 0, -(cos(th)*(u1 + u2))/m, 0, 0, 0, 0, 0, 0, 0, 0, 0, -sin(th)/m, -sin(th)/m]
[ 0, 0, 0, -(sin(th)*(u1 + u2))/m, 0, 0, 0, 0, 0, 0, 0, 0, 0,  cos(th)/m,  cos(th)/m]
[ 0, 0, 0,                      0, 0, 0, 0, 0, 0, 0, 0, 0, 0,       d/I1,      -d/I1]
[ 0, 0, 0,                      0, 0, 0, 0, 0, 0, 0, 0, 0, 0,          0,          0]
[ 0, 0, 0,                      0, 0, 0, 0, 0, 0, 0, 0, 0, 0,          0,          0]
[ 0, 0, 0,                      0, 0, 0, 0, 0, 0, 0, 0, 0, 0,          0,          0]];
    end
    
    function y = output(obj,t,x,u)
      y = [2;x(1:12)]; % horizontal and vertical position of the ball
    end
  end
end
