% Example glue-logic file from which you call your implementation of:
%  (1) Pole Detector
%  (2) Target Detector
%  (3) Planner
%  (4) Controller
%  -------------------
%  (5) SLAM [Note: full implementation is provided]

% Add MRG helper functions
% addpath('mrg'); % COMMENTED OUT

MODE = 1; % 0 for real, 1 for replay, 2 for fake data
ITER = intmax;
husky_id = 2; % Modify for your Husky


if MODE == 0
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
end


P = eye(3); % initialise covariance matrix
x = zeros(3, 1); % init the state vector, first three coords are our pose


if MODE == 1
    filename = '2019-03-06-12-12-27.mat';
    %filename = '2019-03-04-17-27-25.mat';
    collected_data = load(filename);
    scans = collected_data.scans;
    odometries = collected_data.odometries;
    images = collected_data.images;
    ITERS = size(scans, 2);
end

for s = 1:ITERS    
    if MODE == 1
        scan = scans{s};
        od = odometries{s};
        image = images{s};
    else
        mailbox = mexmoos('FETCH');
        scan = GetLaserScans(mailbox, config.laser_channel, true);
        stereo_images = GetStereoImages(mailbox, config.stereo_channel, true);

        od = GetWheelOdometry(mailbox, config.wheel_odometry_channel);
    end
    
    poles = PoleDetector(scan, 800); 
    poles = reshape(cell2mat(poles), 2, []);
    
    ssize = size(od, 2);
    disp(poles);
    3
    
    robot_x = x(1);
    robot_y = x(2);
    yaw = x(3);

    for idx = 1:ssize
        if od(idx).source_timestamp <= scan.timestamp
            alpha = od(idx).yaw;
            robot_x = robot_x*cos(alpha) - sin(alpha)*robot_y + od(idx).x;
            robot_y = robot_x*sin(alpha) + cos(alpha)*robot_y + od(idx).y;
            yaw = yaw + od(idx).yaw;
            %yaw = AngleWrap(yaw);
        end
    end

    dx = robot_x - x(1);
    dy = robot_y - x(2);
    dyaw = yaw - x(3);
    
    
    u = [dx; dy; dyaw];
    
    [x, P] = SLAMUpdate(u, poles, x, P);
                       
    map = reshape(x(4:end), 2, []);
    
    goal_reached = false;
    % goal_reached = ...; % TODO goal reached check
    if goal_reached
        break;
    end

    % target_pose = route_planner(map, x(1:3)); % TODO.
    % velocity, angle = wheel_controller(current_pose, target_pose);
    % SendSpeedCommand(velocity, angle, husky_config.control_channel);

    plot_state(x(1:3), map, poles, images{s}.left.rgb, s, scan);

    pause(0.5);
end


function plot_state(robot_pose, map, poles, image, iter, scan)
    SQUARE_SIZE = 10;
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
    %[mx, my] = pol2cart(map(2, :)' + robot_yaw, map(1, :)');
    %slam_x = map(1, :)*cos(robot_yaw) - sin(robot_yaw)*map(2,:) + robot_x;
    %slam_y = map(1, :)*sin(robot_yaw) + cos(robot_yaw)*map(2,:) + robot_y;           
    scatter(map(1, :), map(2, :))
    

    % plot the tobot
    scatter(robot_x, robot_y, 'red');
    % plot([robot_x, xprime], [robot_y, yprime], 'red');
    
    [px, py] = pol2cart(poles(2, :)' + robot_yaw, poles(1, :)');
    scatter(px + robot_x, py + robot_y, 'magenta');
    ShowLaserScan(scan, [robot_x, robot_y, robot_yaw]');
    axis([-SQUARE_SIZE SQUARE_SIZE -SQUARE_SIZE SQUARE_SIZE])
    axis ij
    axis square
    hold off;

    subplot(1, 2, 2);
    imshow(image);
    title(num2str(iter));
end
