filename = 'eratic1.bag'
bag = rosbag(filename);
tf_select = select(bag, 'Topic', '/tf');
tf_msgs = readMessages(tf_select);

bundle_name = 'bundle1'
tailsitter = 'vicon/MockTS/main'
camera = 'vicon/realsense3/main'

%% Compute mocap_cam_to_ts
% mocap correction
% applied to mocap measurements so that the frame
% aligns with the frame attached to the camera by AprilTag
mocap_camera_rotation_correction = quaternion(rotz(-pi/2), 'rotmat', 'point');
mocap_camera_translation_correction = [15/1000 8/1000 20/1000];
mocap_bundle_rotation_correction = quaternion(roty(pi/2)*rotz(pi/2), 'rotmat', 'point');
if strcmp(bundle_name, 'bundle1')
    mocap_bundle_translation_correction = [-9/1000 410/1000 -40/1000];
elseif strcmp(bundle_name, 'inner_bundle')
    mocap_bundle_translation_correction = [-9/1000 227/1000 -15/1000];
    
end
% because server pull and no occlusion,
% these two have the same timestamp and number of messages
mocap_world_to_ts = get_transforms(tf_msgs, '/world', tailsitter);
mocap_world_to_cam = get_transforms(tf_msgs, '/world', camera);
mocap_world_to_cam = apply_transform(mocap_world_to_cam, ...
                                     mocap_camera_rotation_correction,...
                                     mocap_camera_translation_correction);              
mocap_world_to_ts = apply_transform(mocap_world_to_ts, ...
                                     mocap_bundle_rotation_correction,...
                                     mocap_bundle_translation_correction);
mocap_cam_to_ts = calculate_transform(mocap_world_to_cam, mocap_world_to_ts);

%% Get apriltags_cam_to_ts
apriltags_cam_to_ts = get_transforms(tf_msgs, 'camera', bundle_name);

%% Plot Mocap v.s. Apriltag Euler Angles
mocap_time = [cellfun(@(m) m.timestamp , mocap_world_to_cam)];
t0 = mocap_time(1)
mocap_time = mocap_time - t0;
tag_time = [cellfun(@(m) m.timestamp , apriltags_cam_to_ts)];
tag_time = tag_time-t0;

mocap_ts_to_cam_eul = [cellfun(@(m) quat2eul(m.rotation.conj, 'ZYX'), mocap_cam_to_ts, 'UniformOutput', false)];
mocap_ts_to_cam_eul = [[cellfun(@(m) m(1), mocap_ts_to_cam_eul)] [cellfun(@(m) m(2), mocap_ts_to_cam_eul)] [cellfun(@(m) m(3), mocap_ts_to_cam_eul)]];
apriltags_ts_to_cam_eul = [cellfun(@(m) quat2eul(m.rotation.conj, 'ZYX'), apriltags_cam_to_ts, 'UniformOutput', false)];
apriltags_ts_to_cam_eul = [[cellfun(@(m) m(1), apriltags_ts_to_cam_eul)] [cellfun(@(m) m(2), apriltags_ts_to_cam_eul)] [cellfun(@(m) m(3), apriltags_ts_to_cam_eul)]];

figure
hold on
plot(mocap_time, mocap_ts_to_cam_eul(:,1))
plot(tag_time, apriltags_ts_to_cam_eul(:,1))
legend('mocap', 'apriltag')
title('Camera Yaw')
xlabel('time')
ylabel('rad')

figure
hold on
plot(mocap_time, mocap_ts_to_cam_eul(:,2))
plot(tag_time, apriltags_ts_to_cam_eul(:,2))
legend('mocap', 'apriltag')
title('Camera Pitch')
xlabel('time')
ylabel('rad')

figure
hold on
plot(mocap_time, wrapTo2Pi(mocap_ts_to_cam_eul(:,3)))
plot(tag_time, wrapTo2Pi(apriltags_ts_to_cam_eul(:,3)))
legend('mocap', 'apriltag')
title('Camera Roll')
xlabel('time')
ylabel('rad')

%% Plot Mocap v.s. Camera to TS Position
figure
hold on
plot(mocap_time, [cellfun(@(m) m.translation(1), mocap_cam_to_ts)])
plot(tag_time, [cellfun(@(m) m.translation(1), apriltags_cam_to_ts)])
legend('mocap', 'apriltag')
title('Camera to TS X')
xlabel('time')
ylabel('distance (m)')

figure
hold on
plot(mocap_time, [cellfun(@(m) m.translation(2), mocap_cam_to_ts)])
plot(tag_time, [cellfun(@(m) m.translation(2), apriltags_cam_to_ts)])
legend('mocap', 'apriltag')
title('Camera to TS Y')
xlabel('time')
ylabel('distance (m)')

figure
hold on
plot(mocap_time, [cellfun(@(m) m.translation(3), mocap_cam_to_ts)])
plot(tag_time, [cellfun(@(m) m.translation(3), apriltags_cam_to_ts)])
legend('mocap', 'apriltag')
title('Camera to TS Z')
xlabel('time')
ylabel('distance (m)')

%% Plot some raw data
%plot(mocap_time, [cellfun( @(m) m(3),...
%        [cellfun(@(m) quat2eul(m.rotation, 'ZYX'), mocap_world_to_ts, 'UniformOutput', false)])...
%     ])
%plot(mocap_time, [cellfun(@(m) m.translation(3), mocap_world_to_ts)])
