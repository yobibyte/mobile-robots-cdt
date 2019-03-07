% TODO: route planner, given the map and pose, output target pose

function [prm, path] = RoutePlanner(poles, current_pose, target)

  data = [[current_pose(1), current_pose(2)]; poles; [target(1), target(2)]];
  min_ = min(data);
  max_ = max(data);

  data = data - min_;
  poles = poles - min_;
  current_pose(1) = current_pose(1)-min_(1);
  current_pose(2) = current_pose(2)-min_(2);
  target(1) = target(1)-min_(1);
  target(2) = target(2)-min_(2);

  map = robotics.BinaryOccupancyGrid(ceil(max_(1)-min_(1)),ceil(max_(2)-min_(2)), 100);
  for i=1:size(poles, 1)
      setOccupancy(map,[poles(i, 1) poles(i, 2)], 1)
  end
  inflate(map, 0.5);
  prm = robotics.PRM;
  prm.Map = map;
  prm.NumNodes = 50;
  prm.ConnectionDistance = 5;
  path = findpath(prm, [current_pose(1), current_pose(2)], [target(1), target(2)]);

  %costmap = vehicleCostmap(map);
  %plot(costmap)
  path = path + min_;
end
