% TODO: route planner, given the map and pose, output target pose

function target_pose = RoutePlanner(map, current_pose, target)

  %scatter(map(:,1), map(:,2));
  data = [[current_pose(1), current_pose(2)]; map; [target(1), target(2)]]
  [vx,vy] = voronoi(data(:,1), data(:,2))  
  %[v, c] = voronoin(data)
  v = unique([[vx(1, :); vy(1, :)]';[vx(2, :); vy(2, :)]'], 'rows');
  
  % add the goal and the target too
  %v = [[current_pose(1), current_pose(2)]; v; [target(1), target(2)]];
  
  target_cell = knnsearch(v, [current_pose(1), current_pose(2)]);
  
  
  cla;
  hold  on;
  
  DT = delaunayTriangulation(map(:,1), map(:,2));
  %triplot(DT);
  adj = cell(1, size(vx, 2));
  for i=1:size(vx, 2)
      from = [vx(1, i), vy(1, i)];
      to = [vx(2, i), vy(2, i)];      
      from_idx = knnsearch(v, from);
      to_idx = knnsearch(v, to); 
      if from_idx ~= to_idx
      adj{from_idx} = [adj{from_idx}, to_idx];
      adj{to_idx} = [adj{to_idx}, from_idx];
      end
      plot([vx(1, i), vx(2, i)], [vy(1, i), vy(2, i)],'b-');
      plot([v(from_idx, 1), v(to_idx, 1)], [v(from_idx, 2), v(to_idx, 2)],'b-');
      
      fprintf("%d %d\n", from_idx, to_idx);
  end
  
  plot(map(:,1), map(:,2),'r+',vx,vy,'b-')
  
  scatter([current_pose(1)], [current_pose(2)]);
  scatter([target(1)], [target(2)]);
  xlim([min(map(:,1))-100 max(map(:,1))+100])
  ylim([min(map(:,2))-100 max(map(:,2))+100])
  target_pose = 'ciao'
  
  idx = knnsearch(v, [current_pose(1) current_pose(2)]);
  start_idx = idx;
  target_idx = knnsearch(v, [target(1) target(2)]);
  
  %scatter([v(idx, 1)], [v(idx, 2)], 'g+');
  
  %f = vx(1,:)]; [vy(1,:)
  %t = vx(1,:)]; [vy(1,:)
  
  % compute adjacency
  
  p = PriorityQueue
  p.push(0, idx);  
  prev = v(idx, :);
  visited = containers.Map(-1, -1)
  cameFrom = cell(1, size(vx, 2));
  while (true)
      % expand
      [idx, value] = p.pop()
      
      visited(idx) = true;
      disp(idx);
      from = v(idx, :);
      %plot([prev(1), from(1)], [prev(2), from(2)], 'y-');
      prev = from
      scatter(from(1), from(2));
      
      if idx == target_idx
          disp("YEAH")
          break
      end
      
      for i=1:size(adj{idx}, 2)
          if ~isKey(visited, adj{idx}(i))
              point = v(adj{idx}(i), :);
              g = 0;%value + distance([vx(2, i), vy(2, i)], from);
              h = pdist([point(1), point(2); target(1), target(2)]);
              p.push(g+h, adj{idx}(i));
              cameFrom{adj{idx}(i)} = idx;
          end
      end
%       
%       points = []
%       [row, col] = find(~[(vx(1, :)-from(1)).^2', (vy(1, :)-from(2)).^2'].^.5);
%       rows = unique(row);
%       for idx = 1:numel(rows)
%           points = [points, [vx(2,rows(idx)); vy(2,rows(idx))]];
%       end
%       
%       [row, col] = find(~[(vx(2, :)-from(1)).^2', (vy(2, :)-from(2)).^2'].^.5);
%       rows = unique(row);
%       for idx = 1:numel(rows)
%           points = [points, [vx(1,rows(idx)); vy(1,rows(idx))]];
%       end
%       points = points'
%       for idx = 1:size(points, 1)
%           point = points(idx, :);
%           d = 0;%distance([vx(2, i), vy(2, i)], from);
%           h = pdist([point(1), point(2); target(1), target(2)]);
%           res = find(abs([point(1), point(2)]-v) < 0.01)
%           if size(res) ~= 0
%               if ~isKey(visited, res(1))
%                   p.push((d+h), res(1));
%               end
%           end
%       end
      
%       for i=1:size(vx, 2)
%           
%           %fprintf("A: %f %f %f %f %f\n", from(1), from(2), vx(1, i), vy(1, i), all(from == [vx(1, i) vy(1, i)]));
%           if distance(from, [vx(1, i) vy(1, i)]) < 0.01
%               %scatter(vx(2, i), vy(2, i));
%               d = 0;%distance([vx(2, i), vy(2, i)], from);
%               h = pdist([vx(2, i), vy(2, i); target(1), target(2)]);
%               res = find(abs([vx(2, i), vy(2, i)]-v) < 0.01)
%               if size(res) ~= 0
%                   if ~isKey(visited, res(1))
%                     p.push((d+h), res(1));
%                   end
%               end
%           end
%           %fprintf("B: %f %f %f %f %f\n", from(1), from(2), vx(2, i), vy(2, i), all(from == [vx(2, i) vy(2, i)] ));          
%           if distance(from, [vx(2, i) vy(2, i)]) < 0.01
%               %scatter(vx(1, i), vy(1, i));
%               d = 0; %distance([vx(1, i), vy(1, i)], from);
%               h = pdist([vx(1, i), vy(1, i); target(1), target(2)]);
%               res = find(abs([vx(1, i), vy(1, i)]-v) < 0.01)
%               if size(res) ~= 0
%                   if ~isKey(visited, res(1))
%                     p.push((d+h), res(1));
%                   end
%               end
%           end
%       end
      if p.size == 0
          break;
      end
      %pause(1);
  end
  
  p = target_idx;
  while true      
      pp = cameFrom{p};
      plot([v(p, 1), v(pp, 1)], [v(p, 2), v(pp, 2)], 'r-');
      if pp == start_idx
          break
      end
      p = pp;
  end
end
