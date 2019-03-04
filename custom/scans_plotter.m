
% Files to use:
% '2019-03-04-15-29-48.mat' in movement
% '2019-03-04-15-20-35.mat' static

filename = '2019-03-04-15-29-48.mat';
scans = load(filename,'scans');
scans = scans.scans;

ITERS = size(scans, 1);
% Main loop
for s = 1:ITERS
    % Fetch latest messages from mex-moos

    figure(1);
    ShowThresholdedLaserScan(scans{s});

    pause(0.1); % don't overload moos w/commands
end
