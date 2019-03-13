classdef BnBController < DrakeSystem
  properties
    p
  end
  methods
    function obj = BnBController(plant)
      obj = obj@DrakeSystem(0,0,9,1,true,false);%all true
      obj.p = plant;
      obj = obj.setInputFrame(plant.getOutputFrame);%%CHANGED
      obj = obj.setOutputFrame(plant.getInputFrame);
    end
    
    function u = output(obj,t,~,x)
      mo = x(1);
      q = x(2:5);
      qd = x(6:9);
      
      kp=40; kd=40;
     
      u = 6*obj.p.g-kp*(q(2)-1.5)-kd*(qd(2)-0);

      if t>4.5
          
      elseif t>4
      u = -20;
      
      end

%       load('UT.mat');
%       if t>utraj.tspan(end)
%       u=utraj.eval(utraj.tspan(end));
%       else
%       u=utraj.eval(t);
%       end
      u = max(min(u,100),-100);
    end
  end
  
  methods (Static)
    function [xtraj]=run()
      plant = BnBPlant2D();
      controller = BnBController(plant);
      %v = plant.constructVisualizer;
      sys_closedloop = feedback(plant,controller);
      xfull0=[2; 0; 3; 0; 2.5; 0; 0; 0; -1];
      xtraj=simulate(sys_closedloop,[0 10],xfull0);
      %v.axis = [-4 4 -4 4];
      %playback(v,xtraj);
      
    end
  end
end
