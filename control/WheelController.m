function [angular_velocity, linear_velocity] = WheelController(current_pose, target_pose)
    angular_velocity = (target_pose(3)-current_pose(3))*0.1;
    linear_velocity = (target_pose(1:2)-current_pose(1:2))*0.1;
end

