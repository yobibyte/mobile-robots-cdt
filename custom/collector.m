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
client = ['ExampleCdtClient' num2str(int32(rand*1e7))];
mexmoos('init', 'SERVERHOST', config.host, 'MOOSNAME', client);
mexmoos('REGISTER', config.laser_channel, 0.0);
mexmoos('REGISTER', config.stereo_channel, 0.0);
mexmoos('REGISTER', config.wheel_odometry_channel, 0.0);
pause(3); % Give mexmoos a chance to connect (important!)

ITERS = 100;
scans = cell(1, ITERS);
odometries = cell(1, ITERS);
images = cell(1, ITERS);
% Main loop
for s = 1:ITERS
    % Fetch latest messages from mex-moos
    mailbox = mexmoos('FETCH');

    scan = GetLaserScans(mailbox, config.laser_channel, true);
    scans{s} = scan;

    wheel_odometry = GetWheelOdometry(mailbox, ...
                                      config.wheel_odometry_channel, ...
                                      false);
    odometries{s} = wheel_odometry;

    c_images = GetStereoImages(mailbox, config.stereo_channel, true);
    images{s} = c_images;
    disp(s);
    pause(0.1); % don't overload moos w/commands
end

name = datestr(now,'yyyy-mm-dd-HH-MM-SS');
save(strcat(name, '.mat'), 'scans', 'odometries', 'images');
