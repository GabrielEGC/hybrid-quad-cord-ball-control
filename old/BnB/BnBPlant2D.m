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
      %obj = obj.setInputLimits(-300,300);
      obj = addTransition(obj, ...
        asone_mode, ...            % from mode
        @ForceGuard1, ...                    % q-r<=0 & qdot<=0 %andGuards(obj,g1,g2)
        @collisionDynamics12, ...    % transition method
        true, ...                 % not direct feedthrough %IMPORTANT
        true,flight_mode);%,...                   % time invariant
        %flight_mode);              % to mode 5%%%HUGE _ AVOID ERROR IN HyTO
      obj = addTransition(obj, ...
        flight_mode, ...            % from mode
        andGuards(obj,@LengthGuard,@IncrLengthGuard), ...                    % q-r<=0 & qdot<=0 %andGuards(obj,g1,g2)
        @collisionDynamics21, ...    % transition method
        false, ...                 % not direct feedthrough %IMPORTANT
        true,asone_mode);%,...                   % time invariant
        %asone_mode);              % to mode 
    obj = setSimulinkParam(obj,'InitialStep','1e-3','MaxStep','0.05');
    end

    function [g,dg] = ForceGuard1(obj,t,x,u)
      g = u(1)+eps;  %  CONSIDER +EPS % CONSIDER g1.5=distance 
      dg = [0,0,0,0,0,0,0,0,0,1];
    end
    function [g,dg] = LengthGuard(obj,t,x,u)
      g = eps+obj.l-(x(2)-x(4));
      dg = [0,0,-1,0,1,0,0,0,0,0];
    end
    function [g,dg] = IncrLengthGuard(obj,t,x,u)
      g = eps+(x(8)-x(6));   % theta_st >= 0
      dg = [0,0,0,0,0,0,-1,0,1,0];
    end

    function [xn,m,status,dxn] = collisionDynamics12(obj,m,t,x,u)%[xp,mode,status,dxp]
      xn = x;
      status = 0;                   
      m=2;
      dxndx = eye(8); %8 states
      dxn = [zeros(8,2),dxndx,zeros(8,1)];
    end
    
    function [xn,m,status,dxn] = collisionDynamics21(obj,m,t,x,u)%[xp,mode,status,dxp]
      nvfi = (obj.mq*x(6)+obj.ml*x(8))/(obj.mq+obj.ml);
      xn = [x(1:5);nvfi;x(7);nvfi];
      status = 0;
      m=1;
      dnvfidx68=[(obj.mq)/(obj.mq+obj.ml),0,(obj.ml)/(obj.mq+obj.ml)];
      dxndx = [eye(5),zeros(5,3);zeros(3,5),[dnvfidx68;0,1,0;dnvfidx68]]; %8 states
      dxn = [zeros(8,2),dxndx,zeros(8,1)];
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
