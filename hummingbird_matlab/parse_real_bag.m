%% Open bag and read messages
filename = find_file('.', '.*.bag');
bag = rosbag(filename);
tf_select = select(bag, 'Topic', '/tf');
tf_msgs = readMessages(tf_select);

%% Process bag
% kalman filter estimate
bp = get_transforms(tf_msgs, 'COM', 'partner');
[tbp obp Rbp] = processTransforms(bp);

% raw data
bundle1c_transforms = get_transforms(tf_msgs, 'COM', 'bundle1_corrected_raw');
[bundle1c_time bundle1c_pos bundle1c_rpy] = processTransforms(bundle1c_transforms);

% ground truth
mb_correction_translation = [0 0.32 -0.042];
mb_correction_rotation = [-1,0,0;0,-1,0;0,0,1];
mb = get_transforms(tf_msgs, '/world', 'vicon/STARS_TS3/STARS_TS3');
mb = apply_transform(mb, mb_correction_translation, quaternion(rotm2quat(mb_correction_rotation)));
mp = get_transforms(tf_msgs, '/world', 'vicon/MOCK_TS/MOCK_TS');
N1 = length(mb); 
N2 = length(mp);
i = 1;
j = 1;
k = 1;
bp_true = {};
while and(i <= N1, j <= N2)
    if mb{i}.timestamp == mp{j}.timestamp
        current = {};
        quat = quaternion(mb{i}.rotation)';
        bp_true{k}.translation = quat.rotatepoint(-mb{i}.translation) + quat.rotatepoint(mp{k}.translation);
        new_rotation = quat*quaternion(mp{k}.rotation);
        [w x y z] = parts(new_rotation);
        bp_true{k}.rotation = [w x y z];
        bp_true{k}.timestamp = mb{i}.timestamp;
        k = k+1;
        i = i+1;
        j = j+1;
    elseif mb{i}.timestamp < mp{j}.timestamp
        i = i+1;
    else
        j = j+1;
    end
end

bp_true = bp_true';

[tbp_true obp_true Rbp_true] = processTransforms(bp_true);


%% Plot Position
figure
hold on
plot(tbp_true, obp_true(:,1))
plot(tbp, obp(:,1))
plot(bundle1c_time, bundle1c_pos(:,1))
legend('True base\_link to partner', 'estimated base\_link to partner', 'base\_link to tag')
title('base\_link to partner X offset')
xlabel('time (s)')
ylabel('m')

figure
hold on
plot(tbp_true, obp_true(:,2))
plot(tbp, obp(:,2))
plot(bundle1c_time, bundle1c_pos(:,2))
legend('True base\_link to partner', 'estimated base\_link to partner', 'base\_link to tag')
title('base\_link to partner Y offset')
xlabel('time (s)')
ylabel('m')

figure
hold on
plot(tbp_true, obp_true(:,3))
plot(tbp, obp(:,3))
plot(bundle1c_time, bundle1c_pos(:,3))
legend('True base\_link to partner', 'estimated base\_link to partner', 'base\_link to tag')
title('base\_link to partner Z offset')
xlabel('time (s)')
ylabel('m')

%% Plot rotation
figure
hold on
plot(tbp_true, Rbp_true(:,1))
plot(tbp, Rbp(:,1))
plot(bundle1c_time, bundle1c_rpy(:,1))
legend('True base\_link to partner', 'estimated base\_link to partner', 'base\_link to tag')
title('base\_link to partner roll')
xlabel('time (s)')
ylabel('rads')

figure
hold on
plot(tbp_true, Rbp_true(:,2))
plot(tbp, Rbp(:,2))
plot(bundle1c_time, bundle1c_rpy(:,2))
legend('True base\_link to partner', 'estimated base\_link to partner', 'base\_link to tag')
title('base\_link to partner pitch')
xlabel('time (s)')
ylabel('rads')

figure
hold on
plot(tbp_true, Rbp_true(:,3))
plot(tbp, Rbp(:,3))
plot(bundle1c_time, bundle1c_rpy(:,3))
legend('True base\_link to partner', 'estimated base\_link to partner', 'base\_link to tag')
title('base\_link to partner yaw')
xlabel('time (s)')
ylabel('rads')