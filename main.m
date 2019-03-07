profile on

% Example glue-logic file from which you call your implementation of:
%  (1) Pole Detector
%  (2) Target Detector
%  (3) Planner
%  (4) Controller
%  -------------------
%  (5) SLAM [Note: full implementation is provided]

% Add MRG helper functions
% addpath('mrg'); % COMMENTED OUT

FREQ = 10;
MODE = 1; % 0 for real, 1 for replay, 2 for fake data
ITERS = intmax;
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
    %filename = '2019-03-06-12-12-27.mat';
    filename = '2019-03-06-12-10-43.mat';
    collected_data = load(filename);
    scans = collected_data.scans;
    odometries = collected_data.odometries;
    images = collected_data.images;
    ITERS = size(scans, 2);
end
overall = 0;

controller = WheelController;
goal_reached = false;
goal_pose = [5 0 0];
G_last_global = BuildSE2Transform([0, 0, 0]);

for s = 1:ITERS
    if MODE == 1
        scan = scans{s};
        od = odometries{s};
        image = images{s};
    else
        mailbox = mexmoos('FETCH');
        scan = GetLaserScans(mailbox, config.laser_channel, true);
        image = GetStereoImages(mailbox, config.stereo_channel, true);

        od = GetWheelOdometry(mailbox, config.wheel_odometry_channel);
    end

    poles = PoleDetector(scan, 800);
    poles = reshape(cell2mat(poles), 2, []);

    ssize = size(od, 2);

    G_last = BuildSE2Transform([0, 0, 0]);

    for idx = 1:ssize
        if od(idx).source_timestamp <= scan.timestamp
            G_last_current = BuildSE2Transform([od(idx).x,od(idx).y, od(idx).yaw]);
            G_last = G_last * G_last_current;
        end
    end
    G_t1_t2 = G_last;
    u = SE2ToComponents(G_t1_t2)';

    G_last_global = G_last_global  * G_t1_t2;
    [x, P] = SLAMUpdate(u, poles, x, P);

    map = reshape(x(4:end), 2, []);

    % Check whether we can see the goal, update it (transforming in the global ref system).
    if mod(s, FREQ) == 0
        [visible, goal_z_x] = GoalFinder(image);  % TODO: decrease frequency of goalfinding check
        if visible
          R = [cos(x(3)) -sin(x(3)) 0; sin(x(3)) cos(x(3)) 0; 0 0 1];
          T = [1 0 -x(1); 0 1 -x(2); 0 0 1];
          c_pos = [goal_z_x(1); goal_z_x(2); 1];
          new_pos = T * R * c_pos;
          goal_pose = [new_pos(1)/new_pos(3); new_pos(2)/new_pos(3); 0];
        end
    end

    % Check whether we reached the goal (less than 0.1 distance from its pose).
    if not(goal_reached) && norm(x(1:3) - goal_pose) < 0.1
      goal_reached = true;
    end

    if goal_reached && goal_pose ~= [0 0 0]
        goal_pose = [0 0 0];
    else
        if goal_reached
            break
        end
    end

    if mod(s, FREQ) == 0
        [prm, path] = RoutePlanner(map', x(1:3), goal_pose);
        path = [path, zeros(size(path,1), 1)] % TODO: add goal yaw
    end

    [distance, angular_velocity, linear_velocity, velocity] = controller.update(x(1:3), path(2,:));

    if MODE == 0
        SendSpeedCommand(velocity, angular_velocity, config.control_channel);
        %fprintf("av=%f lv=%f\n", angular_velocity, linear_velocity);
        % target_pose = route_planner(map, x(1:3)); % TODO.
        % velocity, angle = wheel_controller(current_pose, target_pose);
        % SendSpeedCommand(velocity, angle, husky_config.control_channel);
    end

    plot_state(x(1:3), map, poles, image.left.rgb, s, scan, path, goal_pose);
    %figure;
    %subplot(2, 2, 3);
    %show(prm);
    pause(0.01);  %TODO: edit this
end

accumulated_odometry = SE2ToComponents(G_last_global);

function plot_state(robot_pose, map, poles, image, iter, scan, path, goal_pose)
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
    scatter(map(1, :), map(2, :))

    % plot the robot
    scatter(robot_x, robot_y, 'red');
    plot([robot_x, xprime], [robot_y, yprime], 'red');

    % plot the goal
    scatter(goal_pose(1), goal_pose(2), 'green', 'x');

    % plot path
    plot(path(:, 1), path(:, 2))

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
