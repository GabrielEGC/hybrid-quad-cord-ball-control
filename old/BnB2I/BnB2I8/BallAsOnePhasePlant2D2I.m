classdef BallAsOnePhasePlant2D2I < DrakeSystem
  properties
    g = 9.81;  % gravity
    mq = 4;
    ml = 2;
    l = 2;
  end
  
  methods
    function obj = BallAsOnePhasePlant2D2I()
      obj = obj@DrakeSystem(...
        8, ... % number of continuous states
        0, ... % number of discrete states
        2, ... % number of inputs
        9, ... % number of outputs
        false, ... % not direct feedthrough
        true); % time invariant
    end
    
    function [f,df] = dynamics(obj,t,x,u)
        u1=u(1);u2=u(2);
        m=obj.mq;g=obj.g;m2=obj.ml;l=obj.l;
        
        al = pi/2+atan2(x(4)-x(2),x(3)-x(1));
        dal = -(x(4)-x(2))/((x(4)-x(2))^2 + (x(3)-x(1))^2)*(x(7)-x(5))+(x(3)-x(1))/((x(4)-x(2))^2 + (x(3)-x(1))^2)*(x(8)-x(6));
        
        
        ddx=(2*m*u2 + m2*u2 + m2*u2*cos(al)^2 + dal^2*l*m2^2*sin(al) + m2*u1*cos(al)*sin(al) + 2*dal^2*l*m*m2*sin(al))/((m + m2)*(2*m + m2));
        ddy=-(2*g*m^2 - 2*m2*u1 - 2*m*u1 + g*m2^2 + m2*u1*cos(al)^2 - (m2*u2*sin(2*al))/2 + 3*g*m*m2 + dal^2*l*m2^2*cos(al) + 2*dal^2*l*m*m2*cos(al))/((m + m2)*(2*m + m2));
        ddx2=-(m*u2*cos(al)^2 - m2*u2 - 2*m*u2 + 2*dal^2*l*m^2*sin(al) + m*u1*cos(al)*sin(al) + dal^2*l*m*m2*sin(al))/((m + m2)*(2*m + m2));
        ddy2=(m*u1 + m2*u1 - 2*g*m^2 - g*m2^2 + m*u1*cos(al)^2 - (m*u2*sin(2*al))/2 - 3*g*m*m2 + 2*dal^2*l*m^2*cos(al) + dal^2*l*m*m2*cos(al))/((m + m2)*(2*m + m2));
        
        f = [x(5:8);ddx; ddy; ddx2; ddy2];
        df = sparse(8,11);
%         df(1:4,6:9) = eye(4);
%         df(5:8,:) =  [];
    end
    
    function y = output(obj,t,x,u)
      y = [1;x(1:8)]; % horizontal and vertical position of the ball
    end
  end

end
