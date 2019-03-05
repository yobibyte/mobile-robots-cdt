function [coordinates] = PoleDetector(scan, threshold)
    % project polar to cartesian
    angles = -((0:size(scan.ranges, 1)-1).'*-scan.step_size - scan.start_angle + 90) * pi/180;
    coords = [scan.ranges.*cos(angles) scan.ranges.*sin(angles)];
    cla;
    hold on;
    T = coords(find(scan.reflectances > threshold), :);
    X = pdist(T);
    Z = linkage(X);
    
    C = cluster(Z,'cutoff', 0.1,'criterion', 'distance');
    N = length(unique(C));
    coordinates = cell(1, N);
    %ShowLaserScan(scan);
    for i = 1:N
        if size(T(find(C==i), :), 1) > 3
            c = mean(T(find(C==i), :));
            %scatter(c(1), c(2));
            [theta, rho] = cart2pol(c(1), c(2));
            coordinates{i} = [theta, rho];
        end
    end
end

