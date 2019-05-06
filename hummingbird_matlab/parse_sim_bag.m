%
% Variable naming scheme:
%   m - map frame
%   p - partner frame
%   b - base_link frame
filename = 'fwd_backward2.bag'
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
%% 
% compute true base_link to partner
mb = get_transforms(tf_msgs, 'map', 'base_link');
[tmb omb Rmb] = processTransforms(mb);
mp_true = [1 0 1.5];
Rmp_true = [0;0;0]; % roll pitch yaw
obp_true = mp_true - omb;
Rbp_true = [];
for i=1:length(mb)
    Rbp_true = [Rbp_true; rotm2eul(eul2rotm(Rmb(i,:), 'XYZ')'*eul2rotm(Rmp_true', 'XYZ'), 'XYZ')];
end

% base_link to partner estimate
bp = get_transforms(tf_msgs, 'base_link', 'partner');
[tbp obp Rbp] = processTransforms(bp);

% baselink to corrected tag (raw measurement)
bundle1c_transforms = get_transforms(tf_msgs, 'base_link', 'bundle1_corrected_raw');
[bundle1c_time bundle1c_pos bundle1c_rpy] = processTransforms(bundle1c_transforms);


%% Plot above
figure
hold on
% plot map to baselink x vs map to baselink y
% for circle maneuvers
plot(omb(:, 1), omb(:,2))

figure
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
%% Plot IMU data
plot(imuTime, imuAcc(:,1))