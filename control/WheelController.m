classdef WheelController
   properties
      a_pid = PID(0.8, 0, 0.0)
      v_pid = PID(0.5, 0, 0.0)
      lx_pid = PID(0.06, 0, 0.0)
      ly_pid = PID(0.06, 0, 0.0)
   end
   methods
      function [distance, angular_velocity, linear_velocity, velocity] = update(obj, current_pose, target_pose)    ;      
          distance = pdist([current_pose(1), current_pose(2); target_pose(1) target_pose(2)]);
          theta = atan2(target_pose(2)-current_pose(2), target_pose(1)-current_pose(1));
          linear_velocity = [0 0];
          velocity = 0;
          angular_velocity = obj.a_pid.update(current_pose(3), theta);
          if rad2deg(abs(current_pose(3)-theta)) > 30
              velocity = 0
          else
              velocity = -obj.v_pid.update(distance, 0);
          end
          
          linear_velocity = [cos(theta) sin(theta)]*velocity;          
          
          if distance < 0.08             
              % rotate in place
              angular_velocity = obj.a_pid.update(current_pose(3), target_pose(3));              
              velocity = 0;
              linear_velocity = [cos(theta) sin(theta)]*velocity;          
          end

%           if distance > 0.1
%               angular_velocity = obj.a_pid.update(current_pose(3), theta);              
%               if (abs(current_pose(3) - theta) < 0.1)
%                 velocity = -obj.v_pid.update(distance, 0);
%                 linear_velocity = [cos(theta) sin(theta)]*velocity;
%               end
%           else
%               angular_velocity = obj.a_pid.update(current_pose(3), target_pose(3));       
%               velocity = -obj.v_pid.update(distance, 0);
%               linear_velocity = [cos(theta) sin(theta)]*velocity;
%           end          
      end
   end
end