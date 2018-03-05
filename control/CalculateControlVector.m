function [ u ] = CalculateControlVector( delta_left, delta_right, L, angular_slipping)
    if nargin < 4, angular_slipping = 1; end
    
    R = (delta_right + delta_left) / 2;
    theta = (delta_left-delta_right) / (2 * L) / angular_slipping;
    x = R * cos(theta);
    y = R * sin(theta);

    u = [x y theta]';
end
