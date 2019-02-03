%
% Variable naming scheme:
%   m - map frame
%   p - partner frame
%   b - base_link frame
filename = '2019-01-30-19-43-43.bag';
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
mp_true = [2.5 0 1.5];
Rmp_true = [0;0;0]; % roll pitch yaw

% Compute obp_true and Rbp_true
obp_true = mp_true - omb;
Rbp_true = [];
for i=1:length(mb)
    Rbp_true = [Rbp_true; rotm2eul(eul2rotm(Rmb(i,:), 'XYZ')'*eul2rotm(Rmp_true', 'XYZ'), 'XYZ')];
end

tag3c_transforms = get_transforms(tf_msgs, 'base_link', 'tag3_corrected');
[tag3c_time tag3c_pos tag3c_rpy] = processTransforms(tag3c_transforms);
                       
tag4c_transforms = get_transforms(tf_msgs, 'base_link', 'tag4_corrected');
[tag4c_time tag4c_pos tag4c_rpy] = processTransforms(tag4c_transforms);

%% Plot above
hold on
plot(tmb, obp_true(:,1))
plot(tbp, obp(:,1))
plot(tag4c_time, tag4c_pos(:,1))
legend('True base\_link to partner', 'estimated base\_link to partner', 'base\_link to tag4')
title('base\_link to partner X offset')
xlabel('time (s)')
ylabel('m')

figure
hold on
plot(tmb, obp_true(:,2))
plot(tbp, obp(:,2))
plot(tag4c_time, tag4c_pos(:,2))
legend('True base\_link to partner', 'estimated base\_link to partner', 'base\_link to tag4')
title('base\_link to partner Y offset')
xlabel('time (s)')
ylabel('m')

figure
hold on
plot(tmb, obp_true(:,3))
plot(tbp, obp(:,3))
plot(tag4c_time, tag4c_pos(:,3))
legend('True base\_link to partner', 'estimated base\_link to partner', 'base\_link to tag4')
title('base\_link to partner Z offset')
xlabel('time (s)')
ylabel('m')

figure
hold on
plot(tmb, Rbp_true(:,1))
plot(tbp, Rbp(:,1))
plot(tag4c_time, tag4c_rpy(:,1))
legend('True base\_link to partner', 'estimated base\_link to partner', 'base\_link to tag4')
title('base\_link to partner roll')
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
