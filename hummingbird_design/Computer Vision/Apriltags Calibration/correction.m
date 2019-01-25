endtime = 1000;
duration = 1:endtime;

%% Using simple average out calibration
H_t1 = pose_mean(ts_estimate1_poses);
H_t2 = pose_mean(ts_estimate2_poses);
H_t3 = pose_mean(ts_estimate3_poses);
H_t4 = pose_mean(ts_estimate4_poses);
H_gt = pose_mean(ground_truth_poses);
H_cam = pose_mean(r200_location_poses);
H_tag1 = pose_mean(tags3_poses);
H_tag2 = pose_mean(tags4_poses);
H_tag3 = pose_mean(tags5_poses);
H_tag4 = pose_mean(tags6_poses);

% Correct direction of final one
% Rotz = rotz(pi);
% Rotz(4,4) = 1;
% H_gt = H_gt * Rotz;

H_dt1 = H_gt / H_t1;
H_dt2 = H_gt / H_t2;
H_dt3 = H_gt / H_t3;
H_dt4 = H_gt / H_t4;

H_calib1 = H_dt1 * H_t1;
H_calib2 = H_dt2 * H_t2;
H_calib3 = H_dt3 * H_t3;
H_calib4 = H_dt4 * H_t4;

figure;
hold on;
trplot(H_t1, 'frame', 'TSest1', 'length', 0.005)
trplot(H_t2, 'frame', 'TSest2', 'length', 0.005)
trplot(H_t3, 'frame', 'TSest3', 'length', 0.005)
trplot(H_t4, 'frame', 'TSest4', 'length', 0.005)
trplot(H_gt, 'frame', 'Tgt', 'length', 0.005, 'color', 'red')
trplot(H_calib1, 'frame', 'Tcalib', 'length', 0.005, 'color', 'blue')
trplot(H_calib2, 'frame', 'Tcalib', 'length', 0.005, 'color', 'blue')
trplot(H_calib3, 'frame', 'Tcalib', 'length', 0.005, 'color', 'blue')
trplot(H_calib4, 'frame', 'Tcalib', 'length', 0.005, 'color', 'blue')

% trplot(H_cam, 'frame', 'Tcam', 'length', 0.005, 'color', 'black')

grid minor;

%% Draw New Transformation after calibration

ts_estimate1_corrected = zeros(4,4,length(ts_estimate1_poses));
ts_estimate2_corrected = zeros(4,4,length(ts_estimate2_poses));
ts_estimate3_corrected = zeros(4,4,length(ts_estimate3_poses));
ts_estimate4_corrected = zeros(4,4,length(ts_estimate4_poses));

for i = 1:length(ts_estimate1_poses)
    ts_estimate1_corrected(1:4,1:4,i) = H_dt1 * ts_estimate1_poses(:,:,i);
end

for i = 1:length(ts_estimate2_poses)
    ts_estimate2_corrected(:,:,i) = H_dt2 * ts_estimate2_poses(:,:,i);
end

for i = 1:length(ts_estimate3_poses)
    ts_estimate3_corrected(:,:,i) = H_dt3 * ts_estimate3_poses(:,:,i);
end

for i = 1:length(ts_estimate4_poses)
    ts_estimate4_corrected(:,:,i) = H_dt4 * ts_estimate4_poses(:,:,i);
end

figure;
hold on;

plotPoseCloud(ts_estimate1_poses, '.');
plotPoseCloud(ts_estimate1_corrected, '.');
plotPoseCloud(ground_truth_poses,'x');

legend('Original Poses', 'Corrected Poses', 'Ground Truth');
title('Relative Transformations with the Objects');
xlabel('X Axis')
ylabel('Y Axis')
zlabel('Z Axis')
grid on;


subplot(1,2,1);
hold on;

plotPoseCloud(ts_estimate1_poses, '.');
plotPoseCloud(ts_estimate2_poses, '.');
plotPoseCloud(ts_estimate3_poses, '.');
plotPoseCloud(ts_estimate4_poses, '.');
plotPoseCloud(ground_truth_poses,'x');

legend('Tag 1 Estimate Corrected', 'Tag 2 Estimate Corrected', 'Tag 3 Estimate Corrected', 'Tag 4 Estimate Corrected', 'Ground Truth');
title('Tag Poses - Before');
xlabel('X Axis')
ylabel('Y Axis')
zlabel('Z Axis')
grid minor

subplot(1,2,2);
hold on;

plotPoseCloud(ts_estimate1_corrected, '.');
plotPoseCloud(ts_estimate2_corrected, '.');
plotPoseCloud(ts_estimate3_corrected, '.');
plotPoseCloud(ts_estimate4_corrected, '.');
plotPoseCloud(ground_truth_poses,'x');

legend('Tag 1 Estimate Corrected', 'Tag 2 Estimate Corrected', 'Tag 3 Estimate Corrected', 'Tag 4 Estimate Corrected', 'Ground Truth');
title('Tag Poses - After');
xlabel('X Axis')
ylabel('Y Axis')
zlabel('Z Axis')
grid minor

%% ICP Rotate, then correct relative to sensor 2

H_icp12 = pose_icp(ts_estimate2_corrected, ts_estimate1_corrected);
H_icp32 = pose_icp(ts_estimate2_corrected, ts_estimate3_corrected);
H_icp42 = pose_icp(ts_estimate2_corrected, ts_estimate4_corrected);

ts_estimate1_icp_corrected = zeros(4,4,length(ts_estimate1_poses));
ts_estimate3_icp_corrected = zeros(4,4,length(ts_estimate2_poses));
ts_estimate4_icp_corrected = zeros(4,4,length(ts_estimate3_poses));

for i = 1:length(ts_estimate1_poses)
    ts_estimate1_icp_corrected(1:4,1:4,i) = H_icp12 * ts_estimate1_corrected(:,:,i);
end
for i = 1:length(ts_estimate1_poses)
    ts_estimate3_icp_corrected(1:4,1:4,i) = H_icp32 * ts_estimate1_corrected(:,:,i);
end
for i = 1:length(ts_estimate1_poses)
    ts_estimate4_icp_corrected(1:4,1:4,i) = H_icp42 * ts_estimate1_corrected(:,:,i);
end

%% Compare ICP relative corrected point cloud between 2 tags
figure;

subplot(1,2,1);
hold on;
plotPoseCloud(ts_estimate1_corrected, '.');
plotPoseCloud(ts_estimate2_corrected, '.');
plotPoseCloud(ts_estimate3_corrected, '.');
plotPoseCloud(ts_estimate4_corrected, '.');
plotPoseCloud(ground_truth_poses,'x');

legend('Tag 1 Estimate Corrected', 'Tag 2 Estimate Corrected', 'Tag 3 Estimate Corrected', 'Tag 4 Estimate Corrected', 'Ground Truth');
title('Tag Poses - After');
xlabel('X Axis')
ylabel('Y Axis')
zlabel('Z Axis')
grid minor


subplot(1,2,2);
hold on;
plotPoseCloud(ts_estimate2_corrected, '.');
plotPoseCloud(ts_estimate1_icp_corrected, '.');
plotPoseCloud(ts_estimate3_icp_corrected, '.');
plotPoseCloud(ts_estimate4_icp_corrected, '.');
plotPoseCloud(ground_truth_poses,'x');

legend('Tag 1 Estimate Corrected', 'Tag 2 Estimate Corrected', 'Tag 3 Estimate Corrected', 'Tag 4 Estimate Corrected', 'Ground Truth');
title('Tag Poses - After');
xlabel('X Axis')
ylabel('Y Axis')
zlabel('Z Axis')
grid minor

%% Print to calibration variables

H_1 = H_t1 \ H_gt;
H_2 = H_t2 \ H_gt;
H_3 = H_t3 \ H_gt;
H_4 = H_t4 \ H_gt;

T_dt1 = H_1(1:3,4)
Q_dt1 = rotm2quat(H_1(1:3,1:3))

T_dt2 = H_2(1:3,4)
Q_dt2 = rotm2quat(H_2(1:3,1:3))

T_dt3 = H_3(1:3,4)
Q_dt3 = rotm2quat(H_3(1:3,1:3))

T_dt4 = H_4(1:3,4)
Q_dt4 = rotm2quat(H_4(1:3,1:3))
