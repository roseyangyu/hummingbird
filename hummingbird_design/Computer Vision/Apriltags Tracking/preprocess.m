%% Preprocessing

clc
clear

rbag = rosbag('data/nomoving.bag');

bagtf = select(rbag, 'Topic', '/tf','Time',[rbag.StartTime,rbag.StartTime + 5]);
bagimage = select(rbag, 'Topic', '/camera/color/image_raw','Time',[rbag.StartTime,rbag.StartTime + 5]);
imageStructs = readMessages(bagimage,'DataFormat','sensor_msgs/Image');
bagolddetection = select(rbag, 'Topic', '/apriltags2_ros/tag_pose_detections','Time',[rbag.StartTime,rbag.StartTime + 5]);
% tagPoseDetectionStructs = readMessages(bagolddetection,'DataFormat','apriltags2_msgs/AprilTagDetectionPoseArray');

time = zeros(height(bagtf.MessageList), 1);
for i = 1 : height(bagtf.MessageList)
    time(i) = bagtf.MessageList{i,1};
end

% Obtain Tag detections
ts_estimate1 = zeros(height(bagtf.MessageList), 7);
ts_estimate2 = zeros(height(bagtf.MessageList), 7);
ts_estimate3 = zeros(height(bagtf.MessageList), 7);
ts_estimate4 = zeros(height(bagtf.MessageList), 7);
ground_truth = zeros(height(bagtf.MessageList), 7);
r200_location = zeros(height(bagtf.MessageList), 7);
tags3 = zeros(height(bagtf.MessageList), 7);
tags4 = zeros(height(bagtf.MessageList), 7);
tags5 = zeros(height(bagtf.MessageList), 7);
tags6 = zeros(height(bagtf.MessageList), 7);

for i = 1 : length(time)
    disp(i)
    
    tfTime = rostime(time(i) + 1);
    
    if (canTransform(bagtf,'world','ts3', tfTime))
        tf = getTransform(bagtf,'world','ts3', tfTime);
        ts_estimate1(i,1) = tf.Transform.Translation.X;
        ts_estimate1(i,2) = tf.Transform.Translation.Y;
        ts_estimate1(i,3) = tf.Transform.Translation.Z;
        ts_estimate1(i,4) = tf.Transform.Rotation.W;
        ts_estimate1(i,5) = tf.Transform.Rotation.X;
        ts_estimate1(i,6) = tf.Transform.Rotation.Y;
        ts_estimate1(i,7) = tf.Transform.Rotation.Z;
    end
    
    if (canTransform(bagtf,'world','ts4', tfTime))
        tf = getTransform(bagtf,'world','ts4', tfTime);
        ts_estimate2(i,1) = tf.Transform.Translation.X;
        ts_estimate2(i,2) = tf.Transform.Translation.Y;
        ts_estimate2(i,3) = tf.Transform.Translation.Z;
        ts_estimate2(i,4) = tf.Transform.Rotation.W;
        ts_estimate2(i,5) = tf.Transform.Rotation.X;
        ts_estimate2(i,6) = tf.Transform.Rotation.Y;
        ts_estimate2(i,7) = tf.Transform.Rotation.Z;
    end
    
    if (canTransform(bagtf,'world','ts5', tfTime))
        tf = getTransform(bagtf,'world','ts5', tfTime);
        ts_estimate3(i,1) = tf.Transform.Translation.X;
        ts_estimate3(i,2) = tf.Transform.Translation.Y;
        ts_estimate3(i,3) = tf.Transform.Translation.Z;
        ts_estimate3(i,4) = tf.Transform.Rotation.W;
        ts_estimate3(i,5) = tf.Transform.Rotation.X;
        ts_estimate3(i,6) = tf.Transform.Rotation.Y;
        ts_estimate3(i,7) = tf.Transform.Rotation.Z;
    end
    
    if (canTransform(bagtf,'world','ts6', tfTime))
        tf = getTransform(bagtf,'world','ts6', tfTime);
        ts_estimate4(i,1) = tf.Transform.Translation.X;
        ts_estimate4(i,2) = tf.Transform.Translation.Y;
        ts_estimate4(i,3) = tf.Transform.Translation.Z;
        ts_estimate4(i,4) = tf.Transform.Rotation.W;
        ts_estimate4(i,5) = tf.Transform.Rotation.X;
        ts_estimate4(i,6) = tf.Transform.Rotation.Y;
        ts_estimate4(i,7) = tf.Transform.Rotation.Z;
    end
    
    if (canTransform(bagtf,'world','vicon/STARS_TS_Test/STARS_TS_Test', tfTime))
        tf = getTransform(bagtf,'world','vicon/STARS_TS_Test/STARS_TS_Test', tfTime);
        ground_truth(i,1) = tf.Transform.Translation.X;
        ground_truth(i,2) = tf.Transform.Translation.Y;
        ground_truth(i,3) = tf.Transform.Translation.Z;
        ground_truth(i,4) = tf.Transform.Rotation.W;
        ground_truth(i,5) = tf.Transform.Rotation.X;
        ground_truth(i,6) = tf.Transform.Rotation.Y;
        ground_truth(i,7) = tf.Transform.Rotation.Z;
    end
    
    if (canTransform(bagtf,'world','vicon/STARS_R200/STARS_R200', tfTime))
        tf = getTransform(bagtf,'world','vicon/STARS_R200/STARS_R200', tfTime);
        r200_location(i,1) = tf.Transform.Translation.X;
        r200_location(i,2) = tf.Transform.Translation.Y;
        r200_location(i,3) = tf.Transform.Translation.Z;
        r200_location(i,4) = tf.Transform.Rotation.W;
        r200_location(i,5) = tf.Transform.Rotation.X;
        r200_location(i,6) = tf.Transform.Rotation.Y;
        r200_location(i,7) = tf.Transform.Rotation.Z;
    end
    
    if (canTransform(bagtf,'world','tag3', tfTime))
        tf = getTransform(bagtf,'world','tag3', tfTime);
        tags3(i,1) = tf.Transform.Translation.X;
        tags3(i,2) = tf.Transform.Translation.Y;
        tags3(i,3) = tf.Transform.Translation.Z;
        tags3(i,4) = tf.Transform.Rotation.W;
        tags3(i,5) = tf.Transform.Rotation.X;
        tags3(i,6) = tf.Transform.Rotation.Y;
        tags3(i,7) = tf.Transform.Rotation.Z;
    end
    
    if (canTransform(bagtf,'world','tag4', tfTime))
        tf = getTransform(bagtf,'world','tag4', tfTime);
        tags4(i,1) = tf.Transform.Translation.X;
        tags4(i,2) = tf.Transform.Translation.Y;
        tags4(i,3) = tf.Transform.Translation.Z;
        tags4(i,4) = tf.Transform.Rotation.W;
        tags4(i,5) = tf.Transform.Rotation.X;
        tags4(i,6) = tf.Transform.Rotation.Y;
        tags4(i,7) = tf.Transform.Rotation.Z;
    end
    
    if (canTransform(bagtf,'world','tag5', tfTime))
        tf = getTransform(bagtf,'world','tag5', tfTime);
        tags5(i,1) = tf.Transform.Translation.X;
        tags5(i,2) = tf.Transform.Translation.Y;
        tags5(i,3) = tf.Transform.Translation.Z;
        tags5(i,4) = tf.Transform.Rotation.W;
        tags5(i,5) = tf.Transform.Rotation.X;
        tags5(i,6) = tf.Transform.Rotation.Y;
        tags5(i,7) = tf.Transform.Rotation.Z;
    end
    
    if (canTransform(bagtf,'world','tag6', tfTime))
        tf = getTransform(bagtf,'world','tag6', tfTime);
        tags6(i,1) = tf.Transform.Translation.X;
        tags6(i,2) = tf.Transform.Translation.Y;
        tags6(i,3) = tf.Transform.Translation.Z;
        tags6(i,4) = tf.Transform.Rotation.W;
        tags6(i,5) = tf.Transform.Rotation.X;
        tags6(i,6) = tf.Transform.Rotation.Y;
        tags6(i,7) = tf.Transform.Rotation.Z;
    end
end

% Obtain images

clear i bagtf bagimage bagolddetection frames rbag tfTime

%% Analysis Parameters

% endtime = 3;
% duration = 1:endtime;
% 
% ts_estimate1 = ts_estimate1(duration,:);
% ts_estimate2 = ts_estimate2(duration,:);
% ts_estimate3 = ts_estimate3(duration,:);
% ts_estimate4 = ts_estimate4(duration,:);
% tags3 = tags3(duration,:);
% tags4 = tags4(duration,:);
% tags5 = tags5(duration,:);
% tags6 = tags6(duration,:);
% ground_truth = ground_truth(duration,:);
% r200_location = r200_location(duration,:);
% time = time(duration,:);

%% Convert to Homogenenous Matrix

ts_estimate1_poses = toHMatrixRow(ts_estimate1);
ts_estimate2_poses = toHMatrixRow(ts_estimate2);
ts_estimate3_poses = toHMatrixRow(ts_estimate3);
ts_estimate4_poses = toHMatrixRow(ts_estimate4);
ground_truth_poses = toHMatrixRow(ground_truth);
r200_location_poses = toHMatrixRow(r200_location);
tags3_poses = toHMatrixRow(tags3);
tags4_poses = toHMatrixRow(tags4);
tags5_poses = toHMatrixRow(tags5);
tags6_poses = toHMatrixRow(tags6);

function Hrow = toHMatrixRow(poseArray)
    Hrow = zeros(4,4,length(poseArray));
    for i = 1 : length(poseArray)
        Hrow(:,:,i) = toHMatrix(poseArray(i,:));
    end
end

function H = toHMatrix(row)
    H = [quat2rotm(row(4:7)), row(1:3)'; 0,0,0,1];
end
