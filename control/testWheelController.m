
clear all;
s = [0 0 pi/180.0*(45+90)];
e = [10 10 pi/180.0*(-90)];
a = s
controller = WheelController
while true
    cla
    hold on
    disp(a);
    PlotPose(a);
    PlotPose(s);    
    PlotPose(e);

    [distance, angular_velocity, linear_velocity] = controller.update(a, e);

    a(3) = a(3) + angular_velocity;
    a(1:2) = a(1:2) + linear_velocity;
    
    pause(0.1); % don't overload moos w/commands
end