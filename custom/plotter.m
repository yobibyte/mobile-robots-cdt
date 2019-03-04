
% Files to use, with laser scans only (along with neighboring files in terms of date):
% '2019-03-04-15-29-48.mat' in movement
% '2019-03-04-15-20-35.mat' static


% Files to use, with all data.


husky_id = 2; % Modify for your Husky

% Get the channel names and sensor IDs for this Husky
config = GetHuskyConfig(husky_id);

filename = '2019-03-04-17-27-25.mat';
collected_data = load(filename);
scans = collected_data.scans;
odometries = collected_data.odometries;
images = collected_data.images;

ITERS = size(scans, 2)
% Main loop
for s = 1:ITERS
    % Fetch latest messages from mex-moos

    disp(odometries{s})

    % Display laser scan
    subplot(1, 3, 1);
    ShowThresholdedLaserScan(scans{s});

    % Display stereo image
    subplot(1, 3, 2);
    ShowStereoImage(images{s})

    % Display undistorted stereo image
    subplot(1, 3, 3);
    ShowStereoImage(UndistortStereoImage(images{s}, ...
                                         config.camera_model));

    pause(0.1); % don't overload moos w/commands
end
