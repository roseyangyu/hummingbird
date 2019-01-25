function H = pose_icp(poseCloudA, poseCloudB)
    positionListA = squeeze(poseCloudA(1:3,4,:));
    positionListB = squeeze(poseCloudB(1:3,4,:));
    [R,T,ER] = icp(positionListA, positionListB,100);
    
    H = [R,T;[0,0,0,1]];
end