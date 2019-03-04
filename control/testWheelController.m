

s = [0 0 pi/180.0*(45+90)];
e = [10 10 pi/180.0*(-90)];
a = s
while true
    cla
    hold on

    PlotPose(a);
    PlotPose(s);    
    PlotPose(e);

    [angular_velocity, linear_velocity] = WheelController(a, e);

    a(3) = a(3) + angular_velocity;
    a(1:2) = a(1:2) + linear_velocity;
    disp(a(2))
    pause(0.1); % don't overload moos w/commands
end