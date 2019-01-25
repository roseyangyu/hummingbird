
% Starting position
curPos = [2 0 1.3];
curRot = [0.1 0 0];

% Ending position
endPos = [5 3 2.3];
endRot = [0.1-pi 0 0];

waypoints = find_trajectory(curPos, curRot, endPos, endRot, 0.3);
waypoints2 = tailsitter_bezier_path(curPos, curRot, endPos, endRot, 1); % 1 meter away
plot3(waypoints(:,1), waypoints(:,2), waypoints(:,3));
hold on;
plot3(waypoints2(:,1), waypoints2(:,2), waypoints2(:,3));
grid minor
hold off;