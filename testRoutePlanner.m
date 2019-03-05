clear;
hold on;
map = cell(10, 2);
for i=1:10
   map{i} = [random('Normal',0,5) random('Normal',0,5)]; 
end
map = cell2mat(map);
start_pose = [random('Normal',0,5) random('Normal',0,5) 0]
end_pose = [random('Normal',0,5) random('Normal',0,5) 0]
target = RoutePlanner(map,  start_pose, end_pose)
scatter(target(1), target(2));