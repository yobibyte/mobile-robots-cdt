% TODO: goal finder, given the stereo images, return whether we see the goal and where it is x§§

function [visible, target_pose] = GoalFinder(images, current_pose)

  visible = false;
  target_pose = 'ciao';

end


function a = test_target_finder(image)

  match_threshold = 1000;

  n_colors = 10000;
  map = hot(n_colors);
  X = rgb2ind(image, map);

  low = 900;
  high = 3000;
  RoI = roicolor(X,low,high);

  n_ones = sum(RoI(:) == 1)

  %figure(1)
  %imshow(X, map)
  %figure(2)
  %imshow(RoI)

  if n_ones > match_threshold
    a = true
  else
    a = false
  end

end
