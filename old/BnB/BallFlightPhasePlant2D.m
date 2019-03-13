classdef BallFlightPhasePlant2D < DrakeSystem
  properties
    g = 9.81;  % gravity
    mq = 4;
    ml = 2;
  end
  
  methods
    function obj = BallFlightPhasePlant2D()
      obj = obj@DrakeSystem(...
        8, ... % number of continuous states
        0, ... % number of discrete states
        1, ... % number of inputs
        9, ... % number of outputs
        false, ... % not direct feedthrough
        true); % time invariant
    end
    
    function [f,df] = dynamics(obj,t,x,u)
      f = [x(5:8);0; -obj.g+u(1)/obj.mq;0; -obj.g];  % qddot = [0; -g]; x=[q,qdot]
      df = sparse(8,10);
      df(1:4,6:9) = eye(4);
      df(6,10) = 1/obj.mq;
    end
    
    function y = output(obj,t,x,u)
      y = [2;x(1:8)]; % horizontal and vertical position of the ball
    end
  end
end
