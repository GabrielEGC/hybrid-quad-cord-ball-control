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
        10, ... % number of continuous states
        0, ... % number of discrete states
        2, ... % number of inputs
        11, ... % number of outputs
        false, ... % not direct feedthrough
        true); % time invariant
    end
    
    function [f,df] = dynamics(obj,t,x,u)
        u1=u(1);u2=u(2);
        m=obj.mq;g=obj.g;m2=obj.ml;al=x(5);dal=x(10);l=obj.l;
        
        ddx=(2*m*u2 + m2*u2 + m2*u2*cos(al)^2 + dal^2*l*m2^2*sin(al) + m2*u1*cos(al)*sin(al) + 2*dal^2*l*m*m2*sin(al))/((m + m2)*(2*m + m2));
        ddy=-(2*g*m^2 - 2*m2*u1 - 2*m*u1 + g*m2^2 + m2*u1*cos(al)^2 - (m2*u2*sin(2*al))/2 + 3*g*m*m2 + dal^2*l*m2^2*cos(al) + 2*dal^2*l*m*m2*cos(al))/((m + m2)*(2*m + m2));
        ddal=-(u2*cos(al) + u1*sin(al))/(l*(2*m + m2));
        f = [x(6:10);ddx; ddy; 0; 0; ddal];
        df = sparse(10,13);
        df(1:5,7:11) = eye(5);
        df(6:10,:) =  [[ 0, 0, 0, 0, 0, (m2*u1*cos(al)^2 - m2*u1*sin(al)^2 + dal^2*l*m2^2*cos(al) - 2*m2*u2*cos(al)*sin(al) + 2*dal^2*l*m*m2*cos(al))/((m + m2)*(2*m + m2)), 0, 0, 0, 0,  (2*dal*l*sin(al)*m2^2 + 4*dal*l*m*sin(al)*m2)/((m + m2)*(2*m + m2)),        (m2*cos(al)*sin(al))/((m + m2)*(2*m + m2)), (2*m + m2 + m2*cos(al)^2)/((m + m2)*(2*m + m2))]
[ 0, 0, 0, 0, 0,                   (m2*u2*cos(2*al) + dal^2*l*m2^2*sin(al) + 2*m2*u1*cos(al)*sin(al) + 2*dal^2*l*m*m2*sin(al))/((m + m2)*(2*m + m2)), 0, 0, 0, 0, -(2*dal*l*cos(al)*m2^2 + 4*dal*l*m*cos(al)*m2)/((m + m2)*(2*m + m2)), (2*m + 2*m2 - m2*cos(al)^2)/((m + m2)*(2*m + m2)),          (m2*sin(2*al))/(2*(m + m2)*(2*m + m2))]
[ 0, 0, 0, 0, 0,                                                                                                                                   0, 0, 0, 0, 0,                                                                    0,                                                 0,                                               0]
[ 0, 0, 0, 0, 0,                                                                                                                                   0, 0, 0, 0, 0,                                                                    0,                                                 0,                                               0]
[ 0, 0, 0, 0, 0,                                                                                           -(u1*cos(al) - u2*sin(al))/(l*(2*m + m2)), 0, 0, 0, 0,                                                                    0,                           -sin(al)/(l*(2*m + m2)),                         -cos(al)/(l*(2*m + m2))]];
    end
    
    function y = output(obj,t,x,u)
      y = [1;x(1:10)]; % horizontal and vertical position of the ball
    end
  end

end
