% TODO: route planner, given the map and pose, output target pose

function [prm, path] = RoutePlanner(poles, current_pose, target)

  rng(42);

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
  prm = robotics.PRM;
  prm.Map = map;
  prm.NumNodes = 70;
  prm.ConnectionDistance = 5;
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
