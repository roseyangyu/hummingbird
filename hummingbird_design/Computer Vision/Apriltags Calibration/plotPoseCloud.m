function plotPoseCloud(poseCloud, shape)
    scatter3(poseCloud(1,4,:), poseCloud(2,4,:), poseCloud(3,4,:), shape)
end