cla;
hold on;
map = cell(30, 2);
for i=1:30
   map{i} = [random('Normal',0,5) random('Normal',0,5)]; 
end
map = cell2mat(map);
start_pose = [random('Normal',0,5) random('Normal',0,5) 0]
end_pose = [random('Normal',0,5) random('Normal',0,5) 0]
[map, path] = RoutePlanner(map,  start_pose, end_pose)
show(map);
%scatter(path(2, 1), path(2, 2));