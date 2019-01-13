%% Plot location

close all;

figure;
hold on;
plotPoseCloud(ts_estimate1_poses, '.');
plotPoseCloud(ts_estimate2_poses, '.');
plotPoseCloud(ts_estimate3_poses, '.');
plotPoseCloud(ts_estimate4_poses, '.');
plotPoseCloud(r200_location_poses, '.');
plotPoseCloud(ground_truth_poses, 'X');
title('Relative Transformations with the Objects');
legend('TS estimate 1', 'TS estimate 2', 'TS estimate 3', 'TS estimate 4', 'Camera Location', 'Ground Truth');
xlabel('X Axis')
ylabel('Y Axis')
zlabel('Z Axis')
grid on;