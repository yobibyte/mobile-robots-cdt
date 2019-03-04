function [coordinates] = PoleDetector(scan, threshold)
    % project polar to cartesian
    angles = -((0:size(scan.ranges, 1)-1).'*-scan.step_size - scan.start_angle + 90) * pi/180;
    coords = [scan.ranges.*cos(angles) scan.ranges.*sin(angles)];
    
    coordinates = mean(coords(find(scan.reflectances > threshold), :));
end

