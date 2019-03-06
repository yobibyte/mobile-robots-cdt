husky_id = 2; % Modify for your Husky

% Get the channel names and sensor IDs for this Husky
config = GetHuskyConfig(husky_id);

filename = '2019-03-04-17-27-25.mat';
collected_data = load(filename);
scans = collected_data.scans;
odometries = collected_data.odometries;
images = collected_data.images;


for i=1:size(scans, 2)
  PoleDetector(scans{i}, 700);
  [visible, target_pose] = GoalFinder(images{i});


  subplot(1, 2, 2);
  ShowStereoImage(UndistortStereoImage(images{i}, ...
                                       config.camera_model));
  if visible
    disp(visible)
    disp(target_pose)
  end

  subplot(1, 2, 1);
  disp(i);
  pause(0.05);
end
