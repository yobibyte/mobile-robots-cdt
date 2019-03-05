for i=1:size(scans, 2)
PoleDetector(scans{i}, 700);
disp(i);
pause(0.1);
end