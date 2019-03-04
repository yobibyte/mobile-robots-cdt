classdef PID
   properties
      dt = 0.1
      I = 0.0
      prevError = 0
      Kp = 0.06
      Ki = 0.0
      Kd = 0.008
   end
   methods
      function obj = PID(Kp, Ki, Kd)
          obj.Kp = Kp;
          obj.Ki = Ki;
          obj.Kd = Kd;
      end
      function [velocity] = update(obj, current, target)          
          error = (target-current);
          P = error;
          D = (error - obj.prevError)/obj.dt;
          obj.I = obj.I + (error - obj.prevError)*obj.dt/2.0;          
          obj.prevError = error;
          velocity = obj.Kp*P + obj.Ki*obj.I + obj.Kd*D;
      end
   end
end