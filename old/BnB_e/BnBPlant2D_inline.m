classdef BnBPlant2D < HybridDrakeSystem
  properties
    r = 0.5;  % radius of the ball
    e = 0.5;  % coefficient of restitution
    l = 2;
    mq = 4;
    ml = 2;
    g=9.81;
    vtrigg = 0.4;
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
      g4=inline('x(6)-x(8)-obj.vtrigg+eps','obj','t','x','u'); % qdot<=0 %REVIEW EPS
      g5=inline('-(x(6)-x(8)-obj.vtrigg)','obj','t','x','u'); % qdot<=0
      
      obj = addTransition(obj, ...
        asone_mode, ...            % from mode
        g1, ...                    % q-r<=0 & qdot<=0 %andGuards(obj,g1,g2)
        @collisionDynamics12, ...    % transition method
        false, ...                 % not direct feedthrough %IMPORTANT
        true,...                   % time invariant
        flight_mode);              % to mode 
    
    obj = addTransition(obj, ...
        flight_mode, ...            % from mode
        andGuards(obj,g2,g3,g4), ...                    % q-r<=0 & qdot<=0 %andGuards(obj,g1,g2)
        @collisionDynamics21, ...    % transition method
        false, ...                 % not direct feedthrough %IMPORTANT
        true,...                   % time invariant
        asone_mode);              % to mode 
    
    obj = addTransition(obj, ...
        flight_mode, ...            % from mode
        andGuards(obj,g2,g3,g5), ...                    % q-r<=0 & qdot<=0 %andGuards(obj,g1,g2)
        @collisionDynamics22, ...    % transition method
        false, ...                 % not direct feedthrough %IMPORTANT
        true,...                   % time invariant
        flight_mode);              % to mode 
    end

    function [xn,m,status] = collisionDynamics12(obj,m,t,x,u)
      xn = x;
      disp(x)
      status = 0;                   %m ??????????????
      m=2;                          %REVIEWWWWWWW HUGEEEEEEEE
    end
    
    function [xn,m,status] = collisionDynamics21(obj,m,t,x,u)
      nvfi = (obj.mq*x(6)+obj.ml*x(8))/(obj.mq+obj.ml);
      xn = [x(1:5);nvfi;x(7);nvfi];
      status = 0;                   %m ??????????????
      m=1;
    end
    
    function [xn,m,status] = collisionDynamics22(obj,m,t,x,u)
        ml=obj.ml;mq=obj.mq;e=obj.e;
        v2i=x(8);
        v1i=x(6);
        v2f=((ml*v2i+mq*v1i)-mq*e*(v2i-v1i))/(mq+ml);
        v1f=v2f+e*(v2i-v1i);
      xn = [x(1:5);v1f;x(7);v2f];
      status = 0;                   %m ??????????????
      m=2;
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
