%% Plot position_setpoint_triplet_step
t1 = positionsetpointtripletstep0.timestamp;
xyz1 = [positionsetpointtripletstep0.currentx,...
       positionsetpointtripletstep0.currenty,...
       positionsetpointtripletstep0.currentz];
vel1 = [positionsetpointtripletstep0.currentvx,...
       positionsetpointtripletstep0.currentvy,...
       positionsetpointtripletstep0.currentvz];
% plot x vs y graph
figure(1)
plot(xyz1(:,1), xyz1(:,2))
figure(2)
plot(t1, vel1(:,1))
%% Plot position_setpoint_triplet
t2 = positionsetpointtriplet0.currenttimestamp;
xyz2 = [positionsetpointtriplet0.currentx,...
       positionsetpointtriplet0.currenty,...
       positionsetpointtriplet0.currentz];
vel2 = [positionsetpointtriplet0.currentvx,...
       positionsetpointtriplet0.currentvy,...
       positionsetpointtriplet0.currentvz];
% plot x vs y graph
figure(1)
hold on
plot(xyz2(:,1), xyz2(:,2))
figure(2)
hold on
plot(t2, vel2(:,1))

%% Plot actuator outputs
filename = find_file('.', '.*actuator_outputs.*')
actuatoroutputs0 = readtable(filename);
t = actuatoroutputs0.timestamp;
hold on
subplot(2,2,1)
plot(t, actuatoroutputs0.output_0_)
title('motor left')
subplot(2,2,2)
plot(t, actuatoroutputs0.output_1_)
title('motor right')
subplot(2,2,3)
plot(t, actuatoroutputs0.output_4_)
title('elevon left')
subplot(2,2,4)
plot(t, actuatoroutputs0.output_5_)
title('elevon right')

%% Plot virtual actuator outputs
filename = find_file('.', '.*actuator_outputs_virtual.*')
actuatoroutputsvirtual = readtable(filename);
t = actuatoroutputsvirtual.timestamp;
hold on
subplot(2,2,1)
plot(t, actuatoroutputsvirtual.output_0_)
title('motor left')
subplot(2,2,2)
plot(t, actuatoroutputsvirtual.output_1_)
title('motor right')
subplot(2,2,3)
plot(t, actuatoroutputsvirtual.output_4_)
title('elevon left')
subplot(2,2,4)
plot(t, actuatoroutputsvirtual.output_5_)
title('elevon right')

%% Attitude setpoints versus measured versus mocap
filename = find_file('.', '.*vehicle_attitude_setpoint.*')
vehicleattitudesetpoints = readtable(filename);
filename = find_file('.', '.*control_state.*')
controlstate = readtable(filename);
filename = find_file('.', '.*att_pos_mocap.*')
attposmocap = readtable(filename);

t1 = vehicleattitudesetpoints.timestamp;
t2 = controlstate.timestamp;
t3 = attposmocap.timestamp;

rpy1 = quat2eul([vehicleattitudesetpoints.q_d_0_ vehicleattitudesetpoints.q_d_1_ vehicleattitudesetpoints.q_d_2_, vehicleattitudesetpoints.q_d_3_], 'XYZ'); 
rpy = quat2eul([controlstate.q_0_ controlstate.q_1_ controlstate.q_2_, controlstate.q_3_], 'XYZ');
rpy2 = quat2eul([attposmocap.q_0_ attposmocap.q_1_ attposmocap.q_2_, attposmocap.q_3_], 'XYZ');

figure
subplot(1,4,1)
plot(t1, rpy1(:,1))
hold on
plot(t2, rpy(:,1))
plot(t3, rpy2(:,1))
title('roll setpoint')
legend('setpoint', 'measured', 'mocap')
subplot(1,4,2)
plot(t1, rpy1(:,2))
hold on
plot(t2, rpy(:,2))
plot(t3, rpy2(:,2))
title('pitch setpoint')
legend('setpoint', 'measured', 'mocap')
subplot(1,4,3)
plot(t1, rpy1(:,3))
hold on
plot(t2, rpy(:,3))
plot(t3, rpy2(:,3))
title('yaw setpoint')
legend('setpoint', 'measured', 'mocap')
subplot(1,4,4)
plot(t1, vehicleattitudesetpoints.thrust)
title('thrust')

%% Plot body rotation rate setpoints v.s. actual
figure
filename = find_file('.', '.*vehicle_rates_setpoint.*')
vehicleratessetpoints = readtable(filename);
filename = find_file('.', '.*control_state.*')
controlstate = readtable(filename);

t = vehicleratessetpoints.timestamp;
t2 = controlstate.timestamp;

subplot(1,4,1)
plot(t, vehicleratessetpoints.roll)
hold on
plot(t2, controlstate.roll_rate)
title('roll')
legend('setpoint', 'measured')

subplot(1,4,2)
plot(t, vehicleratessetpoints.pitch)
hold on
plot(t2, controlstate.pitch_rate)
title('pitch')
legend('setpoint', 'measured')

subplot(1,4,3)
plot(t, vehicleratessetpoints.yaw)
hold on
plot(t2, controlstate.yaw_rate)
title('yaw')
legend('setpoint', 'measured')

subplot(1,4,4)
plot(t, vehicleratessetpoints.thrust)
title('thrust')
