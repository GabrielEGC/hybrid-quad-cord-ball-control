classdef BallFlightPhasePlant2D2I < DrakeSystem
  properties
    g = 9.81;  % gravity
    mq = 4;
    ml = 2;
    l = 2;
  end
  
  methods
    function obj = BallFlightPhasePlant2D2I()
      obj = obj@DrakeSystem(...
        10, ... % number of continuous states
        0, ... % number of discrete states
        2, ... % number of inputs
        11, ... % number of outputs
        false, ... % not direct feedthrough
        true); % time invariant
    end
    
    function [f,df] = dynamics(obj,t,x,u)
      f = [x(6:10);u(2)/obj.mq; -obj.g+u(1)/obj.mq;0; -obj.g;0];  % qddot = [0; -g]; x=[q,qdot]
      df = sparse(10,13);
      df(1:5,7:11) = eye(5);
      df(6,13) = 1/obj.mq;
      df(7,12) = 1/obj.mq;
    end
    
    function y = output(obj,t,x,u)
      y = [2;x(1:10)]; % horizontal and vertical position of the ball
    end
  end
end
