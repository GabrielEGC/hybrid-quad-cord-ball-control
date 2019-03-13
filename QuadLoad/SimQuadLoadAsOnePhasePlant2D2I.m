classdef SimQuadLoadAsOnePhasePlant2D2I < DrakeSystem
  properties
    l = 2;
    mq = 4;
    ml = 1;
    g = 9.81;
    I1 = 2;
    d = 0.25;
  end
  
  methods
    function obj = SimQuadLoadAsOnePhasePlant2D2I()
      obj = obj@DrakeSystem(...
        12, ... % number of continuous states
        0, ... % number of discrete states
        2, ... % number of inputs
        12, ... % number of outputs
        false, ... % not direct feedthrough
        true); % time invariant
      %obj = setStateFrame(obj,CoordinateFrame('PPTrajectoryOutput',12,'y'));
      obj = obj.setInputLimits(200*[-1;-1],200*[1;1]);
    end
    
    function [f,df] = dynamics(obj,t,x,u)
        u1=u(1);u2=u(2);
        m=obj.mq;g=obj.g;m2=obj.ml;l=obj.l;d=obj.d;I1=obj.I1;
        al=x(6);dal=x(12);th=x(3);
        
        ddx=-(2*m*u1*sin(th) + 2*m*u2*sin(th) + (3*m2*u1*sin(th))/2 + (3*m2*u2*sin(th))/2 - (m2*u1*sin(2*al - th))/2 - (m2*u2*sin(2*al - th))/2 - dal^2*l*m2^2*sin(al) - 2*dal^2*l*m*m2*sin(al))/((m + m2)*(2*m + m2));
        ddy=-(2*g*m^2 + g*m2^2 - 2*m*u1*cos(th) - 2*m*u2*cos(th) - (3*m2*u1*cos(th))/2 - (3*m2*u2*cos(th))/2 + (m2*u1*cos(2*al - th))/2 + (m2*u2*cos(2*al - th))/2 + 3*g*m*m2 + dal^2*l*m2^2*cos(al) + 2*dal^2*l*m*m2*cos(al))/((m + m2)*(2*m + m2));
        ddth=d/I1*(u1-u2);
        ddal=-(sin(al - th)*(u1 + u2))/(l*(2*m + m2));
        
        f = [x(7:12);ddx; ddy; ddth; 0; 0; ddal];
        df = sparse(12,15);
        df(1:6,8:13) = eye(6);
        df(7:12,:) =  [[ 0, 0, 0, -((u1 + u2)*(m2*cos(2*al - th) + 4*m*cos(th) + 3*m2*cos(th)))/(4*m^2 + 6*m*m2 + 2*m2^2), 0, 0, (m2*(u1*cos(2*al - th) + u2*cos(2*al - th) + 2*dal^2*l*m*cos(al) + dal^2*l*m2*cos(al)))/((m + m2)*(2*m + m2)), 0, 0, 0, 0, 0,  (2*dal*l*m2*sin(al))/(m + m2), -(2*m*sin(th) - (m2*sin(2*al - th))/2 + (3*m2*sin(th))/2)/((m + m2)*(2*m + m2)), -(2*m*sin(th) - (m2*sin(2*al - th))/2 + (3*m2*sin(th))/2)/((m + m2)*(2*m + m2))]
[ 0, 0, 0, -((u1 + u2)*(m2*sin(2*al - th) + 4*m*sin(th) + 3*m2*sin(th)))/(4*m^2 + 6*m*m2 + 2*m2^2), 0, 0, (m2*(u1*sin(2*al - th) + u2*sin(2*al - th) + 2*dal^2*l*m*sin(al) + dal^2*l*m2*sin(al)))/((m + m2)*(2*m + m2)), 0, 0, 0, 0, 0, -(2*dal*l*m2*cos(al))/(m + m2),  (2*m*cos(th) - (m2*cos(2*al - th))/2 + (3*m2*cos(th))/2)/((m + m2)*(2*m + m2)),  (2*m*cos(th) - (m2*cos(2*al - th))/2 + (3*m2*cos(th))/2)/((m + m2)*(2*m + m2))]
[ 0, 0, 0,                                                                                       0, 0, 0,                                                                                                             0, 0, 0, 0, 0, 0,                              0,                                                                            d/I1,                                                                           -d/I1]
[ 0, 0, 0,                                                                                       0, 0, 0,                                                                                                             0, 0, 0, 0, 0, 0,                              0,                                                                               0,                                                                               0]
[ 0, 0, 0,                                                                                       0, 0, 0,                                                                                                             0, 0, 0, 0, 0, 0,                              0,                                                                               0,                                                                               0]
[ 0, 0, 0,                                                 (cos(al - th)*(u1 + u2))/(l*(2*m + m2)), 0, 0,                                                                      -(cos(al - th)*(u1 + u2))/(l*(2*m + m2)), 0, 0, 0, 0, 0,                              0,                                                    -sin(al - th)/(l*(2*m + m2)),                                                    -sin(al - th)/(l*(2*m + m2))]];
    end
    
    function y = output(obj,t,x,u)
      y = x; % horizontal and vertical position of the ball
    end
  end

end
