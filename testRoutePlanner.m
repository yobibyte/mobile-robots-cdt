clear;
hold off;
map = cell(10, 2);
for i=1:10
   map{i} = [random('Normal',0,50) random('Normal',0,50)]; 
end
map = cell2mat(map);

RoutePlanner(map, [0 -100 0], [0 100 0])