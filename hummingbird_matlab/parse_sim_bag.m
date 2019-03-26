%
% Variable naming scheme:
%   m - map frame
%   p - partner frame
%   b - base_link frame
%filename = '2019-03-10-19-35-53.bag'; % forward and backward
%filename = '2019-03-14-21-22-46.bag' % just go up
%filename = '2019-03-14-21-44-58.bag'
%filename = '2019-03-14-22-26-16.bag'
%filename = '2019-03-14-22-41-05.bag'
%filename = '2019-03-14-23-30-50.bag'
%filename = '2019-03-15-12-50-49.bag'
filename = '2019-03-15-13-56-15.bag'
%filename = '2019-03-15-14-06-38.bag'
bag = rosbag(filename);
tf_select = select(bag, 'Topic', '/tf');
tf_msgs = readMessages(tf_select);
%% Model states
% This retrieves gazebo's simtime and model state.
% I've found that it's close enough to the map->base_link transformation.
modelStatesSel = select(bag, 'Topic', '/gazebo/model_states');
modelStateMsgs = readMessages(modelStatesSel);
gazeboTimeSel = select(bag, 'Topic', '/clock');
gazeboTimeMsgs = readMessages(gazeboTimeSel);
simTime = linspace(gazeboTimeMsgs{1}.Clock_.Sec + gazeboTimeMsgs{1}.Clock_.Nsec/1000000000, ...
                   gazeboTimeMsgs{end}.Clock_.Sec + gazeboTimeMsgs{end}.Clock_.Nsec/1000000000, ...
                   length(modelStateMsgs));
hb_fixed = 2;
hb = 3;
[hb_pos, hb_rpy] = processGazeboStates(modelStateMsgs, hb);
%% map to partner
mp = get_transforms(tf_msgs, 'map', 'partner');
[tmp omp Rmp] = processTransforms(mp);

mb = get_transforms(tf_msgs, 'map', 'base_link');
[tmb omb Rmb] = processTransforms(mb);

bp = get_transforms(tf_msgs, 'base_link', 'partner');
[tbp obp Rbp] = processTransforms(bp);

% compute true values
% NOTE: need to manually set true map to partner values
mp_true = [1.5 0 1.5];
Rmp_true = [0;0;0]; % roll pitch yaw

% Compute obp_true and Rbp_true
obp_true = mp_true - omb;
Rbp_true = [];
for i=1:length(mb)
    Rbp_true = [Rbp_true; rotm2eul(eul2rotm(Rmb(i,:), 'XYZ')'*eul2rotm(Rmp_true', 'XYZ'), 'XYZ')];
end

bundle1_transforms = get_transforms(tf_msgs, 'camera', 'bundle1');
[bundle1_time bundle1_pos bundle1_rpy] = processTransforms(bundle1_transforms);

bundle1c_transforms = get_transforms(tf_msgs, 'base_link', 'bundle1_corrected');
[bundle1c_time bundle1c_pos bundle1c_rpy] = processTransforms(bundle1c_transforms);


% compute bundle1 in base_link frame
baseLinkToCamera = [0;-0.046;0.384];
R = [0 0 1; -1 0 0; 0 -1 0]; % converts base_link frame to camera frame

bundle1_baselink_pos = [];
bundle1_baselink_rpy = [];
for i=1:numel(bundle1_time)
    bundle1_baselink_pos = [bundle1_baselink_pos; (baseLinkToCamera + R*bundle1_pos(i,:)')'];
    ypr = rotm2eul(R*eul2rotm([bundle1_rpy(i,3) bundle1_rpy(i,2) bundle1_rpy(i,1)]));
    bundle1_baselink_rpy = [bundle1_baselink_rpy; [ypr(3) ypr(2) ypr(1)]];
end


%% Plot above
hold on
plot(tmb, obp_true(:,1))
plot(tbp, obp(:,1))
plot(bundle1c_time, bundle1c_pos(:,1))
legend('True base\_link to partner', 'estimated base\_link to partner', 'base\_link to tag')
title('base\_link to partner X offset')
xlabel('time (s)')
ylabel('m')

figure
hold on
plot(tmb, obp_true(:,2))
plot(tbp, obp(:,2))
plot(bundle1c_time, bundle1c_pos(:,2))
legend('True base\_link to partner', 'estimated base\_link to partner', 'base\_link to tag')
title('base\_link to partner Y offset')
xlabel('time (s)')
ylabel('m')

figure
hold on
plot(tmb, obp_true(:,3))
plot(tbp, obp(:,3))
plot(bundle1c_time, bundle1c_pos(:,3))
legend('True base\_link to partner', 'estimated base\_link to partner', 'base\_link to tag')
title('base\_link to partner Z offset')
xlabel('time (s)')
ylabel('m')

figure
hold on
plot(tmb, Rbp_true(:,1))
plot(tbp, Rbp(:,1))
plot(bundle1c_time, bundle1c_rpy(:,1))
legend('True base\_link to partner', 'estimated base\_link to partner', 'base\_link to tag')
title('base\_link to partner roll')
xlabel('time (s)')
ylabel('rads')

figure
hold on
plot(tmb, Rbp_true(:,2))
plot(tbp, Rbp(:,2))
plot(bundle1c_time, bundle1c_rpy(:,2))
legend('True base\_link to partner', 'estimated base\_link to partner', 'base\_link to tag')
title('base\_link to partner pitch')
xlabel('time (s)')
ylabel('rads')

figure
hold on
plot(tmb, Rbp_true(:,3))
plot(tbp, Rbp(:,3))
plot(bundle1c_time, bundle1c_rpy(:,3))
legend('True base\_link to partner', 'estimated base\_link to partner', 'base\_link to tag')
title('base\_link to partner yaw')
xlabel('time (s)')
ylabel('rads')
%% Plot above on single figure
figure
subplot(2,2,1)
hold on
plot(tmb, obp_true(:,1))
plot(tbp, obp(:,1))
plot(tag4c_time, tag4c_pos(:,1))
legend('True base\_link to partner', 'estimated base\_link to partner', 'base\_link to tag4')
title('base\_link to partner X offset')
xlabel('time (s)')
ylabel('m')

subplot(2,2,2)
hold on
plot(tmb, obp_true(:,2))
plot(tbp, obp(:,2))
plot(tag4c_time, tag4c_pos(:,2))
legend('True base\_link to partner', 'estimated base\_link to partner', 'base\_link to tag4')
title('base\_link to partner Y offset')
xlabel('time (s)')
ylabel('m')

subplot(2,2,3)
hold on
plot(tmb, obp_true(:,3))
plot(tbp, obp(:,3))
plot(tag4c_time, tag4c_pos(:,3))
legend('True base\_link to partner', 'estimated base\_link to partner', 'base\_link to tag4')
title('base\_link to partner Z offset')
xlabel('time (s)')
ylabel('m')

subplot(2,2,4)
hold on
plot(tmb, Rbp_true(:,3))
plot(tbp, Rbp(:,3))
plot(tag4c_time, tag4c_rpy(:,3))
legend('True base\_link to partner', 'estimated base\_link to partner', 'base\_link to tag4')
title('base\_link to partner yaw')
xlabel('time (s)')
ylabel('rads')


figure
hold on
plot(tmb, Rbp_true(:,2))
plot(tbp, Rbp(:,2))
plot(tag4c_time, tag4c_rpy(:,2))
legend('True base\_link to partner', 'estimated base\_link to partner', 'base\_link to tag4')
title('base\_link to partner pitch')
xlabel('time (s)')
ylabel('rads')


%% Plot bundle1 raw measurements
figure
hold on
% plot raw baselink to bundle1
plot(bundle1_time, bundle1_pos(:,1))
plot(bundle1_time, bundle1_pos(:,2))
plot(bundle1_time, bundle1_pos(:,3))
legend('x', 'y', 'z')
title('raw bundle1 pos measurements')
figure
plot(bundle1_time, bundle1_rpy(:,1))
title('raw bundle1 roll measurement')
figure
plot(bundle1_time, bundle1_rpy(:,2))
title('raw bundle1 pitch measurement')
figure
plot(bundle1_time, bundle1_rpy(:,3))
title('raw bundle1 yaw measurement')
%% Plot bundle1 raw measurements tranformed to baselink frame
%tmb omb Rmb

figure
hold on
% plot raw baselink to bundle1
plot(bundle1_time, bundle1_baselink_pos(:,1))
plot(bundle1_time, bundle1_baselink_pos(:,2))
plot(bundle1_time, bundle1_baselink_pos(:,3))
legend('x', 'y', 'z')
title('raw bundle1_baselink pos measurements')

figure
plot(bundle1_time, bundle1_baselink_rpy(:,1))
title('raw bundle1\_baselink roll measurements')

figure
plot(bundle1_time, bundle1_baselink_rpy(:,2))
title('raw bundle1\_baselink pitch measurements')

figure
plot(bundle1_time, bundle1_baselink_rpy(:,3))
title('raw bundle1\_baselink yaw measurements')

%% Extract imu data
imuSel = select(bag,'Topic','/mavros/imu/data');
imuMsgs = readMessages(imuSel);
imuTime = [cellfun(@(m) m.Header.Stamp.Sec +m.Header.Stamp.Nsec/10^9, imuMsgs)];
imuAcc = [cellfun(@(m) m.LinearAcceleration.X, imuMsgs),...
               cellfun(@(m) m.LinearAcceleration.Y, imuMsgs),...
               cellfun(@(m) m.LinearAcceleration.Z, imuMsgs)];
imuAng = [cellfun(@(m) m.AngularVelocity.X, imuMsgs),...
               cellfun(@(m) m.AngularVelocity.Y, imuMsgs),...
               cellfun(@(m) m.AngularVelocity.Z, imuMsgs)];
