classdef BnBPlant2D < HybridDrakeSystem
  properties
    r = 0.5;  % radius of the ball
    cor = .4;  % coefficient of restitution
    l = 2;
    mq = 4;
    ml = 2;
    g=9.81;
  end
  
  methods
    function obj = BnBPlant2D()
      obj = obj@HybridDrakeSystem(...
        1, ...  % number of inputs
        9);     % number of outputs
      
      % create flight mode system
      sys = BallAsOnePhasePlant2D();
      obj = setInputFrame(obj,sys.getInputFrame);
      obj = setOutputFrame(obj,sys.getOutputFrame);
      
      sys2 = BallFlightPhasePlant2D();
      sys2 = setInputFrame(sys2,sys.getInputFrame);
      sys2 = setOutputFrame(sys2,sys.getOutputFrame);
      
      [obj,asone_mode] = addMode(obj,sys);  % add the 1st mode
      [obj,flight_mode] = addMode(obj,sys2);  % add the 2nd mode
      
      g1=inline('u(1)+eps','obj','t','x','u');  % CONSIDER +EPS % CONSIDER g1.5=distance
      
      g2=inline('obj.l-(x(2)-x(4))','obj','t','x','u'); % qdot<=0
      g3=inline('x(8)-x(6)','obj','t','x','u'); % qdot<=0
      
      obj = addTransition(obj, ...
        asone_mode, ...            % from mode
        g1, ...                    % q-r<=0 & qdot<=0 %andGuards(obj,g1,g2)
        @collisionDynamics12, ...    % transition method
        false, ...                 % not direct feedthrough %IMPORTANT
        true,...                   % time invariant
        flight_mode);              % to mode 
    
    obj = addTransition(obj, ...
        flight_mode, ...            % from mode
        andGuards(obj,g2,g3), ...                    % q-r<=0 & qdot<=0 %andGuards(obj,g1,g2)
        @collisionDynamics21, ...    % transition method
        false, ...                 % not direct feedthrough %IMPORTANT
        true,...                   % time invariant
        asone_mode);              % to mode 
    end

    function [xn,m,status] = collisionDynamics12(obj,m,t,x,u)
      xn = x;
      status = 0;                   %m ??????????????
      m=2;                          %REVIEWWWWWWW HUGEEEEEEEE
      %xn = [x(1:3); -obj.cor*x(4)];      % qdot = -cor*qdot
%     if (xn(4)<0.01) status = 1; % stop simulating if ball has stopped
%     else status = 0; end
    end
    
    function [xn,m,status] = collisionDynamics21(obj,m,t,x,u)
      nvfi = (obj.mq*x(6)+obj.ml*x(8))/(obj.mq+obj.ml);
      xn = [x(1:5);nvfi;x(7);nvfi];
      status = 0;                   %m ??????????????
      m=1;
      %xn = [x(1:3); -obj.cor*x(4)];      % qdot = -cor*qdot
%     if (xn(4)<0.01) status = 1; % stop simulating if ball has stopped
%     else status = 0; end
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
