function dist = EuclidianDistance(p1, p2)
dist = sqrt(sum((p1 - p2).^2));
end