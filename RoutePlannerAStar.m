% TODO: route planner, given the map and pose, output target pose

function [prm, path] = RoutePlanner(poles, current_pose, target)
  
  data = [[current_pose(1), current_pose(2)]; poles; [target(1), target(2)]];
  min_ = min(data)-[1, 1];
  max_ = max(data)+[1, 1];

  data = data - min_;
  poles = poles - min_;
  current_pose(1) = current_pose(1)-min_(1);
  current_pose(2) = current_pose(2)-min_(2);
  target(1) = target(1)-min_(1);
  target(2) = target(2)-min_(2);

  map = robotics.BinaryOccupancyGrid(ceil(max_(1)-min_(1)),ceil(max_(2)-min_(2)), 10);
  for i=1:size(poles, 1)
      setOccupancy(map,[poles(i, 1) poles(i, 2)], 1)
  end
  old_map = copy(map);
  inflate(map, 0.5);
  
  start = world2grid(map, current_pose(1:2))
  goal_register = robotics.BinaryOccupancyGrid(ceil(max_(1)-min_(1)),ceil(max_(2)-min_(2)), 10);
  setOccupancy(goal_register, [target(1) target(2)], 1)
  OptimalPath=astar(start(1), start(2), occupancyMatrix(map), occupancyMatrix(goal_register), 8)
  
  cla;
  hold on;
  show(map)
  %grid_current = world2grid(map, current_pose(1:2));
  %grid_poles = world2grid(map, poles);
  %scatter(grid_poles(:, 1), grid_poles(:, 2), 'x');
  %scatter(grid_current(1), grid_current(2), 'x');
  
  optimal_path = grid2world(map, OptimalPath);
  
  plot(optimal_path(1,2),optimal_path(1,1),'o','color','k')
  plot(optimal_path(end,2),optimal_path(end,1),'o','color','b')
  plot(optimal_path(:,2),optimal_path(:,1),'r')
  %legend('Goal','Start','Path')
  hold off;
  prm = robotics.PRM;
  prm.Map = map;
  prm.NumNodes = 70;
  prm.ConnectionDistance = 15;
  %if (getOccupancy(map, [target(1), target(2)]))
  try
    path = findpath(prm, [current_pose(1), current_pose(2)], [target(1), target(2)]);
  catch
    inflate(map, 0.1);
    prm = robotics.PRM;
    prm.Map = old_map;
    prm.NumNodes = 50;
    prm.ConnectionDistance = 5;
    path = findpath(prm, [current_pose(1), current_pose(2)], [target(1), target(2)]);
  end


  %costmap = vehicleCostmap(map);
  %plot(costmap)
  path = path + min_;
end
