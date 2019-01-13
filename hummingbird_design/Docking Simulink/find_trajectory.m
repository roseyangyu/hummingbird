function [waypoints] = find_trajectory(currentPos, currentRot, endPos, endRot, dist, facePose)
    
    startx = currentPos(1);
    starty = currentPos(2);
    startz = currentPos(3);
    startyaw = currentRot(3);
    startPoint = [startx starty startz startyaw];
    
    endx = endPos(1);
    endy = endPos(2);
    endz = endPos(3);
    endyaw = endRot(3);
    endPoint = [endx endy endz endyaw];
    
    % Rise to the desired height first
    midPoint1 = [startx starty endz startyaw];
    verticalSpeed = 0.3; % meters per second
    if (endz ~= startz)
        traj1 = changePoseTrajectory(startPoint, midPoint1, 0.01, (endz - startz) / verticalSpeed);
    else
        traj1 = midPoint1;
    end
    
    % Do a simple bezier curve to get to the final location
    xstartdelta = cos(startyaw) * dist;
    ystartdelta = sin(startyaw) * dist;
    
    xenddelta = cos(endyaw) * dist;
    yenddelta = sin(endyaw) * dist;
    
    midPoint2 = [startx + xstartdelta starty + ystartdelta endz startyaw];
    midPoint3 = [endx + xenddelta endy + yenddelta endz endyaw];
    
    % Visualize
%     cla
%     placelabel(midPoint1(1:2),'pt_1');
%     placelabel(midPoint2(1:2),'pt_2');
%     placelabel(midPoint3(1:2),'pt_3');
%     placelabel(endPoint(1:2),'pt_4');
%     axis equal;
%     grid minor;

    xyspeed = 0.3;
    t = linspace(0,1,101);
    [pts, angles, totaldist] = bezier(t, midPoint1(1:2), midPoint2(1:2), midPoint3(1:2), endPoint(1:2));
    
%     hold on
%     plot(pts(:,1),pts(:,2))
%     hold off

    totalpoints = totaldist / xyspeed * 100;
    if(totalpoints > 5000)
        totalpoints = 5000;
    end
    tt = linspace(0,1,totalpoints);
    traj2 = spline(t, pts', tt)';
    anglesnormalized = spline(t, angles', tt)';
    
    if(nargin == 6)
        ydelta = -traj2(:,2) + facePose(2);
        xdelta = -traj2(:,1) + facePose(1);
        yaws = atan2(ydelta, xdelta);
        yaws(end,:) = yaws(end-1,:);
    else
        yaws = atan2(anglesnormalized(:,2), anglesnormalized(:,1));
    end
    
    endzCol = endz * ones(length(yaws),1);
    traj2Final = [traj2 endzCol yaws];
    
    waypoints = [traj1; traj2Final];

end