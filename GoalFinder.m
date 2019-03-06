% TODO: goal finder, given the stereo images, return whether we see the goal and where it is x§§

function [visible, target_pose] = GoalFinder(images)

  husky_id = 2; % Modify for your Husky

  % Get the channel names and sensor IDs for this Husky
  config = GetHuskyConfig(husky_id);

  images = UndistortStereoImage(images, config.camera_model);
  [left_is_goal, left_goal_coordinates] = target_finder(images.left.rgb);
  [right_is_goal, right_goal_coordinates] = target_finder(images.right.rgb);
  visible = left_is_goal && right_is_goal;
  if not(visible)
    target_pose = [0 0];
    return
  end


  left_x = left_goal_coordinates(1) - config.camera_model.left.cx;
  right_x = right_goal_coordinates(1) - config.camera_model.right.cx;
  z = config.camera_model.left.fx * config.camera_model.baseline / (left_x - right_x);
  x = (left_x + right_x) * config.camera_model.baseline / (2*(left_x - right_x));

  target_pose = [z x];

end


function [is_goal, goal_coordinates] = target_finder(image)
  % given an image, find the goal and return its coordinates
  % is_goal, bool, whether the taget is there
  % goal_coordinates, the coordinates (meaningful ony if is_goal is true)

  match_threshold = 800;

  n_colors = 10000;
  map = hot(n_colors);
  X = rgb2ind(image, map);

  low = 1300;
  high = 4000;
  RoI = roicolor(X,low,high);

  n_ones = sum(RoI(:) == 1);

  %figure(1)
  %imshow(X, map)
  %figure
  %imshow(RoI)

  cluster_data = [];
  for i = 1:size(RoI,1)
    for j = 1:size(RoI,2)
      if RoI(i, j) == 1
        cluster_data = [cluster_data; [j, i]];
      end
    end
  end
  k=2; %Number of classes
  [idx,centroid] = kmeans(cluster_data,k, 'Replicates', 5);
  c_dist = pdist(centroid,'euclidean');
  if c_dist < 130
     goal_coordinates = (centroid(1, :) + centroid(2, :))/2;
  else
    size_1 = size(cluster_data(idx==1), 1);
    size_2 = size(cluster_data(idx==2), 1);
    if size_1 >= size_2
      goal_coordinates = centroid(1, :);
    else
      goal_coordinates = centroid(2, :);
    end
  end

  goal_coordinates = round(goal_coordinates);

  %marked = insertMarker(image,centroid,'x', 'size',10);
  %figure
  %imshow(marked)

  if n_ones > match_threshold
    is_goal = true;
  else
    is_goal = false;
  end

end







%% Get depth, v1.


% Camera parameters definition.
%intrinsic_matrix = [config.camera_model.left.fx, 0, 0; 0, config.camera_model.left.fy, 0; config.camera_model.left.cx, config.camera_model.left.cy, 1];
%cameraParams = cameraParameters('IntrinsicMatrix', intrinsic_matrix);

%stereo_rotation = eye(3);
%stereo_translation = [config.camera_model.baseline, 0, 0];
%stereoParams = stereoParameters(cameraParams, cameraParams, stereo_rotation, stereo_translation);


%[frameLeftRect, frameRightRect] = rectifyStereoImages(images.left.rgb, images.right.rgb, stereoParams);
%frameLeftGray  = rgb2gray(frameLeftRect);
%frameRightGray = rgb2gray(frameRightRect);

%disparityMap = disparity(frameLeftGray, frameRightGray);
%figure;
%imshow(disparityMap, [0, 64]);
%title('Disparity Map');
%colormap jet
%colorbar

%points3D = reconstructScene(disparityMap, stereoParams)

%points3D = points3D ./ 1000;  % convert to meters

%left_goal_coordinates
%points3D(left_goal_coordinates)
% Find the distances from the camera in meters.
%dists = sqrt(sum(points3D(left_goal_coordinates) .^ 2))


%% Get depth, v2.

% Camera parameters definition.
%K = [config.camera_model.left.fx 0 0; 0 config.camera_model.left.fy 0; config.camera_model.left.cx config.camera_model.left.cy 1];
%P = K * [eye(3) zeros(3, 1)];

%stereo_rotation = eye(3);
%stereo_translation = [config.camera_model.baseline; 0; 0];
%P_prime = K * [stereo_rotation stereo_translation];

%A = [left_goal_coordinates(1) * P(3, :) - P(1, :); left_goal_coordinates(2) * P(3, :) - P(2, :); right_goal_coordinates(1) * P_prime(3, :) - P_prime(1, :); right_goal_coordinates(2) * P_prime(3, :) - P_prime(2, :)];
%reconstruction = A \ zeros(4, 1)
