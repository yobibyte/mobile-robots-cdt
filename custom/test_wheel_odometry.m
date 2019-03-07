
% Files to use, with laser scans only (along with neighboring files in terms of date):
% '2019-03-04-15-29-48.mat' in movement
% '2019-03-04-15-20-35.mat' static


% Files to use, with all data.


husky_id = 2; % Modify for your Husky

% Get the channel names and sensor IDs for this Husky
config = GetHuskyConfig(husky_id);

filename = '2019-03-06-12-10-43.mat';
collected_data = load(filename);
scans = collected_data.scans;
odometries = collected_data.odometries;
images = collected_data.images;

ITERS = size(scans, 2)

x = 0
y = 0

for i = 1:ITERS

    od = odometries{i};
    for j = 1:size(od, 2)
      alpha = od(j).yaw;
      R = [cos(alpha) -sin(alpha) 0; sin(alpha) cos(alpha) 0; 0 0 1];
      T = [1 0 od(j).x; 0 1 od(j).y; 0 0 1];
      c_pos = [x; y; 1];
      new_pos = T * R' * c_pos;
      x = new_pos(1) / new_pos(3);
      y = new_pos(2) / new_pos(3);
      hold on
      scatter(x, y)
    end

end
