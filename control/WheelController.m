classdef WheelController
   properties
      a_pid = PID(0.06, 0, 0.008)
      lx_pid = PID(0.06, 0, 0.008)
      ly_pid = PID(0.06, 0, 0.008)
   end
   methods
      function [angular_velocity, linear_velocity] = update(obj, current_pose, target_pose)          
          linear_velocity(1) = obj.lx_pid.update(current_pose(1), target_pose(1));
          linear_velocity(2) = obj.ly_pid.update(current_pose(2), target_pose(2));
          angular_velocity = obj.a_pid.update(current_pose(3), target_pose(3));       
          
      end
   end
end

%function [angular_velocity, linear_velocity] = WheelController(current_pose, target_pose)
%    angular_velocity = (target_pose(3)-current_pose(3))*0.1;
%    linear_velocity = (target_pose(1:2)-current_pose(1:2))*0.1;
%end