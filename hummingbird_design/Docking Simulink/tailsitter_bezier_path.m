function trajectory = tailsitter_bezier_path(currentPos, currentRot, tsPos, tsRot, distFromApriltag)

    % Adjust End Position to be slightly before it
    endYaw = tsRot(3);
    xDelta = cos(endYaw) * distFromApriltag;
    yDelta = sin(endYaw) * distFromApriltag;
    
    endPos = tsPos;
    
    endPos(1) = endPos(1) + xDelta;
    endPos(2) = endPos(2) + yDelta;
    
    trajectory = find_trajectory(currentPos, currentRot, endPos, tsRot, 0.8, tsPos);
end