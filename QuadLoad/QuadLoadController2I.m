classdef QuadLoadController2I < DrakeSystem
  properties
    p
  end
  methods
    function obj = QuadLoadController2I(plant)
      obj = obj@DrakeSystem(0,0,13,2,true,true);%all true
      obj.p = plant;
      obj = obj.setInputFrame(plant.getOutputFrame);%%CHANGED
      obj = obj.setOutputFrame(plant.getInputFrame);
    end
    
    function u = output(obj,t,~,x)
      mo = x(1);
      q = x(2:7);
      qd = x(8:13);
      
      kp=20; kd=20;
     
%       u(1) = 6*obj.p.g-kp*(q(2)-6)-kd*(qd(2)-0);
%       u(2) = -kp*(q(1)-6)-kd*(qd(1)-0);

      load('UTPCdone.mat');
      if t>utraj.tspan(end)
      u=utraj.eval(utraj.tspan(end));
      else
      u=utraj.eval(t);
      end
%       u(1) = 5*obj.p.g/2;
%       u(2) = 5*obj.p.g/2;    
%     if t>2.5
%         u(1)=-100;u(2)=-100;
%     end
      
      u(1) = max(min(u(1),200),-200);
      u(2) = max(min(u(2),200),-200);
    end
  end
  
  methods (Static)
    function [xtraj]=run()
%       plant = BnBPlant2D();
%       controller = BnBController(plant);
%       %v = plant.constructVisualizer;
%       sys_closedloop = feedback(plant,controller);
%       xfull0=[2; 0; 3; 0; 2.5; 0; 0; 0; -1];
%       xtraj=simulate(sys_closedloop,[0 10],xfull0);
%       %v.axis = [-4 4 -4 4];
%       %playback(v,xtraj);
      
    end
  end
end
