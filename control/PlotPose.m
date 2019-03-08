function [ ] = PlotPose( pose )
% pose 1 and 2 are coordinates
% pose 3 is angle
start = [pose(1) pose(2)];
theta = pose(3);
scatter([pose(1)], [pose(2)]);

ends = start + [cos(theta) sin(theta)];
quiver(start(:,1), start(:,2), 1*cos(theta), 1*sin(theta));

end