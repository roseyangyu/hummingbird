%
% Variable naming scheme:
%   m - map frame
%   p - partner frame
%   b - base_link frame
filename = '2019-01-27-23-56-50.bag';
bag = rosbag(filename);
tf_select = select(bag, 'Topic', '/tf');
tf_msgs = readMessages(tf_select);
%% Model states
modelStatesSel = select(bag, 'Topic', '/gazebo/model_states');
modelStateMsgs = readMessages(modelStatesSel);
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
plot(tmb, obp_true(:,1))
hold on
plot(tbp, obp(:,1))
plot(tag4c_time, tag4c_pos(:,1))
legend('True base\_link to partner', 'estimated base\_link to partner', 'base\_link to tag4')
title('base\_link to partner X offset')
figure
plot(tmb, obp_true(:,2))
hold on
plot(tbp, obp(:,2))
plot(tag4c_time, tag4c_pos(:,2))
legend('True base\_link to partner', 'estimated base\_link to partner', 'base\_link to tag4')
title('base\_link to partner Y offset')
figure
plot(tmb, obp_true(:,3))
hold on
plot(tbp, obp(:,3))
plot(tag4c_time, tag4c_pos(:,3))
legend('True base\_link to partner', 'estimated base\_link to partner', 'base\_link to tag4')
title('base\_link to partner Z offset')
figure
hold on
plot(tmb, Rbp_true(:,1))
plot(tag4c_time, tag4c_rpy(:,1))
legend('True base\_link to partner', 'estimated base\_link to partner', 'base\_link to tag4')
title('base\_link to partner roll')
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

