% TODO: goal finder, given the stereo images, return whether we see the goal and where it is x§§

function [visible, target_pose] = GoalFinder(images, current_pose)

  visible = false;
  target_pose = 'ciao';

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

  n_ones = sum(RoI(:) == 1)

  %figure(1)
  %imshow(X, map)
  %figure(2)
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
  [idx,centroid] = kmeans(cluster_data,k);
  c_dist = pdist(centroid,'euclidean')
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

  %marked = insertMarker(X,goal_coordinates,'x', 'size',10);
  %figure(3)
  %imshow(marked, map)

  if n_ones > match_threshold
    is_goal = true
  else
    is_goal = false
  end

end
