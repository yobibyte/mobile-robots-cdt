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

filename = '2019-03-04-17-27-25.mat';
collected_data = load(filename);
scans = collected_data.scans;
odometries = collected_data.odometries;
images = collected_data.images;
ITERS = size(scans, 2);

P = eye(3); % initialise covariance matrix
x = zeros(3, 1); % init the state vector, first three coords are our pose

for s = 1:ITERS
    scan = scans{s};
    poles = PoleDetector(scan, 1000);
    % TODO Is the dimensionality below right?
    poles = reshape(cell2mat(poles), [], 2)';
    
    
    od = odometries{s};
    u = [od.x; od.y; od.yaw];
    [x, P] = SLAMUpdate(u, poles, x, P);
    
    % current_pose = x(1, :); % TODO
    % map = x(2:end, :); % TODO
    %TODO: add check on goal detection somewhere
    %target_pose = route_planner(map, current_pose); % TODO.
    %velocity, angle = wheel_controller(current_pose, target_pose);
    %SendSpeedCommand(velocity, angle, husky_config.control_channel);
    clf();
    subplot(1, 3, 1);
    
    if size(x, 1) > 3
        map = reshape(x(4:end), [], 2);
        
        k = tan(x(3));
        l = x(2) - k*x(1);
        yprime = k * (x(1)+0.1) + l;
        
        hold on;
        scatter(map(:, 1), map(:, 2));
        scatter(x(1), x(2), 'red');
        plot([x(1), x(1)+0.1],[x(2), yprime], 'red');
        hold off;
        
    end
    
     subplot(1, 4, 3);
     imshow(images{s}.left.rgb)
     title(num2str(s));
     
     subplot(1, 4, 4);
     
     
     
     % plot poles
     [px, py] = pol2cart(poles(1, :)', poles(2, :)' + x(3));
     
     scatter(px - x(1), py - x(2))

     
    pause(0.5);
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
