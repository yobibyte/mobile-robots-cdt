% Example glue-logic file from which you call your implementation of:
%  (1) Pole Detector
%  (2) Target Detector
%  (3) Planner
%  (4) Controller
%  -------------------
%  (5) SLAM [Note: full implementation is provided]

% Add MRG helper functions
% addpath('mrg'); % COMMENTED OUT

husky_id = 2; % Modify for your Husky

% Get the channel names and sensor IDs for this Husky
config = GetHuskyConfig(husky_id);

% Initialise mex-moos and register channels
% clear mexmoos % not needed b/c matlab will check if it has already been
                % initalised UNLESS you changed one of the five lines below
% client = ['ExampleCdtClient' num2str(int32(rand*1e7))];
% mexmoos('init', 'SERVERHOST', config.host, 'MOOSNAME', client);
% mexmoos('REGISTER', config.laser_channel, 0.0);
% mexmoos('REGISTER', config.stereo_channel, 0.0);
% mexmoos('REGISTER', config.wheel_odometry_channel, 0.0);
% pause(3); % Give mexmoos a chance to connect (important!)

%filename = '2019-03-04-17-27-25.mat';
filename = '2019-03-04-17-27-25.mat';
filename = 'stolen_data.mat';
collected_data = load(filename);
scans = collected_data.scans;
odometries = collected_data.odometries;
images = collected_data.images;
ITERS = size(scans, 2);

P = eye(3); % initialise covariance matrix
x = zeros(3, 1); % init the state vector, first three coords are our pose

for s = 1:ITERS
    scan = scans{s};
    poles = PoleDetector(scan, 800);
    poles = reshape(cell2mat(poles), [], 2)';
    
    od = odometries{s};
    u = [od.x; od.y; od.yaw];
    [x, P] = SLAMUpdate(u, poles, x, P);
    
    map = reshape(x(4:end), [], 2);
    plot_state(x(1:3), map, poles, images{s}.left.rgb, s);
    
    pause(0.4);
end
% TODO sync wheelodpometry frequency sampling
% TODO 
function plot_state(robot_pose, map, poles, image, iter)
    clf();
    subplot(1, 2, 1);

    robot_x = robot_pose(1);
    robot_y = robot_pose(2);
    robot_yaw = robot_pose(3);

    k = tan(robot_yaw);
    l = robot_y - k*robot_x;
    xprime = robot_x+0.5;
    yprime = k * (xprime) + l;

    hold on;

    % plot the slam state
    [mx, my] = pol2cart(map(:, 2)' + robot_yaw, map(:, 1)');
    scatter(mx + robot_x, my + robot_y)

    % plot the tobot
    scatter(robot_x, robot_y, 'red');
    plot([robot_x, xprime],[robot_y, yprime], 'red');

    % plot poles
    [px, py] = pol2cart(poles(2, :)' + robot_yaw, poles(1, :)');
    scatter(px + robot_x, py + robot_y, 'magenta');

    axis([-5 5 -5 5])
    axis ij
    axis square
    hold off;
    
    subplot(1, 2, 2);
    imshow(image)
    title(num2str(iter));
end
% while true
    % Fetch latest messages from mex-moos
    %mailbox = mexmoos('FETCH');
    %scan = GetLaserScans(mailbox, config.laser_channel, true);
    %stereo_images = GetStereoImages(mailbox, config.stereo_channel, true);
    %wheel_odometry = GetWheelOdometry(mailbox, ...
                                      %config.wheel_odometry_channel, ...
                                      %true);
    %poles = PoleDetector(scan, 800);
    %[x_new, P] = SLAMUpdate(wheel_odometry, poles, x, P);

    %current_pose = x_new(1, :); % TODO
    %map = x_new(2:end, :); % TODO
    %target_pose = route_planner(map, current_pose); % TODO.

    %velocity, angle = wheel_controller(current_pose, target_pose);
    %SendSpeedCommand(velocity, angle, husky_config.control_channel);

% end
