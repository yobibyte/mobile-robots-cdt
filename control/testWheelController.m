husky_id = 2; % Modify for your Husky

% Get the channel names and sensor IDs for this Husky
config = GetHuskyConfig(husky_id);

% Initialise mex-moos and register channels
clear mexmoos % not needed b/c matlab will check if it has already been
                % initalised UNLESS you changed one of the five lines below
client = ['ExampleCdtClient' num2str(int32(rand*1e7))];
mexmoos('init', 'SERVERHOST', config.host, 'MOOSNAME', client);
mexmoos('REGISTER', config.laser_channel, 0.0);
mexmoos('REGISTER', config.stereo_channel, 0.0);
mexmoos('REGISTER', config.wheel_odometry_channel, 0.0);
pause(3); % Give mexmoos a chance to connect (important!)
SendSpeedCommand(0, 0, config.control_channel)

s = [0 0 deg2rad(0)];
e = [3 2 deg2rad(180)];
a = s
controller = WheelController
SendSpeedCommand(0, 0, config.control_channel);

cla
hold on
disp(a);
scatter(e(1), e(2), 'x');
while true
    
    %PlotPose(a);
    %PlotPose(s);    
    %PlotPose(e);
    
    mailbox = mexmoos('FETCH');
    od = GetWheelOdometry(mailbox, config.wheel_odometry_channel, false);
    
    % update s
    G_last = BuildSE2Transform(a);
    for idx = 1:size(od, 2)
        G_last_current = BuildSE2Transform([od(idx).x,od(idx).y, od(idx).yaw]);
        G_last = G_last * G_last_current;
    end
    
    a = SE2ToComponents(G_last);
    %a = a + u;
    disp(a);
    
    [distance, angular_velocity, linear_velocity, velocity] = controller.update(a, e);
    velocity = min(velocity, 0.5);
    angular_velocity = min(angular_velocity, 0.5);
    fprintf("Angular velocity %f\n", angular_velocity);
    SendSpeedCommand(velocity, angular_velocity, config.control_channel);
    
    if (angular_velocity ~= 0)
        
    end
    
    scatter(a(1), a(2));
    
    %a(3) = a(3) + angular_velocity;
    %a(1:2) = a(1:2) + linear_velocity;
    
    pause(0.01); % don't overload moos w/commands
end