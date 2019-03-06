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
    ShowLaserScan(scan);
    for i = 1:N
        if size(T(find(C==i), :), 1) > 1
            c = mean(T(find(C==i), :));
            %fprintf("C=%d N=%d\n", i, size(T(find(C==i), :)));
            
            idx = rangesearch(coords, c, 0.5);
            
            v = var(coords(idx{1},:));
            %disp(i);
            %disp(v);
            %scatter(c(1), c(2));
            %fprintf("%d %d\n", size(idx{1}, 2), size(T(find(C==i), :), 2));
            %for j=1:size(idx{1}, 2)
                %scatter(coords(idx{1}(j), 1), coords(idx{1}(j), 2));
            %end
            if v(1) < 0.004 && v(2) < 0.004
                scatter(c(1), c(2));
                [theta, rho] = cart2pol(c(1), c(2));
                coordinates{i} = [rho, theta];
            end            
        end
    end
end

