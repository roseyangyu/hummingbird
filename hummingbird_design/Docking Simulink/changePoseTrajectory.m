function traj = changePoseTrajectory(startPose, endPose, Ts, duration) 
    t = [0 duration];
    
    if (duration > 10)
        duration = 10;
    end
    tsmooth = 0:Ts:duration; % 3 seconds
    trajectoryPoints = [startPose; endPose];
    traj = spline(t, trajectoryPoints', tsmooth)';
end