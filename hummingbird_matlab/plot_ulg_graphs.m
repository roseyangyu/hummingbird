%% Vehicle status, vehicle control mode
filename = find_file('.', '.*vehicle_status.*');
vehiclestatus = readtable(filename);

filename = find_file('.', '.*control_mode.*');
vehiclecontrolmode = readtable(filename);

filename = find_file('.', '.*commander_state.*');
commanderstate = readtable(filename);

%% Show commands
filename = find_file('.', '.*vehicle_command.*');
vehiclecommand = readtable(filename);
%% Plot position_setpoint_triplet vs local position vs mocap
filename = find_file('.', '.*position_setpoint_triplet.*')
positionsetpointtriplet = readtable(filename);

filename = find_file('.', '.*vehicle_local_position_setpoint.*')
vehiclelocalpositionsetpoint = readtable(filename);

filename = find_file('.', '.*vehicle_local_position_0.csv')
vehiclelocalposition = readtable(filename);

filename = find_file('.', '.*att_pos_mocap.*')
attposmocap = readtable(filename);

t1 = vehiclelocalpositionsetpoint.timestamp;
t2 = vehiclelocalposition.timestamp;
t3 = attposmocap.timestamp;

% plot z tracking
figure
subplot(1,3,1)
plot(t1, vehiclelocalpositionsetpoint.x)
hold on
plot(t2, vehiclelocalposition.x)
plot(t3, attposmocap.x)
plot(t1, vehiclelocalpositionsetpoint.acc_x)
plot(t1, vehiclelocalpositionsetpoint.vx)
plot(t2, vehiclelocalposition.vx)
legend('setpoint x', 'estimated x', 'mocap', 'ax des', 'vx setpoint', 'estimated vx')

subplot(1,3,2)
plot(t1, vehiclelocalpositionsetpoint.y)
hold on
plot(t2, vehiclelocalposition.y)
plot(t3, attposmocap.y)
plot(t1, vehiclelocalpositionsetpoint.acc_y)
plot(t1, vehiclelocalpositionsetpoint.vy)
plot(t2, vehiclelocalposition.vy)
legend('setpoint y', 'estimated y', 'mocap', 'ay des', 'vy setpoint', 'estimated vy')

subplot(1,3,3)
plot(t1, vehiclelocalpositionsetpoint.z)
hold on
plot(t2, vehiclelocalposition.z)
plot(t3, attposmocap.z)
plot(t1, vehiclelocalpositionsetpoint.acc_z)
plot(t1, vehiclelocalpositionsetpoint.vz)
plot(t2, vehiclelocalposition.vz)
legend('setpoint z', 'estimated z', 'mocap', 'az des', 'vz setpoint', 'estimated vz')

%% Plot actuator outputs
filename = find_file('.', '.*actuator_outputs.*')
actuatoroutputs0 = readtable(filename);
t = actuatoroutputs0.timestamp;
figure
plot(t, actuatoroutputs0.output_0_)
hold on
plot(t, actuatoroutputs0.output_1_)
legend('motor left', 'motor right')
figure
plot(t, actuatoroutputs0.output_4_)
hold on
plot(t, actuatoroutputs0.output_5_)
legend('elevon left', 'elevon right')

%% Plot virtual actuator outputs
filename = find_file('.', '.*actuator_outputs_virtual.*')
actuatoroutputsvirtual = readtable(filename);
t = actuatoroutputsvirtual.timestamp;
figure
plot(t, actuatoroutputsvirtual.output_0_)
hold on
plot(t, actuatoroutputsvirtual.output_1_)
legend('motor left', 'motor right')
figure
plot(t, actuatoroutputsvirtual.output_4_)
hold on
plot(t, actuatoroutputsvirtual.output_5_)
legend('elevon left', 'elevon right')

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

rpy1 = quat2eul([vehicleattitudesetpoints.q_d_0_ vehicleattitudesetpoints.q_d_1_ vehicleattitudesetpoints.q_d_2_, vehicleattitudesetpoints.q_d_3_], 'ZYX'); 
rpy = quat2eul([controlstate.q_0_ controlstate.q_1_ controlstate.q_2_, controlstate.q_3_], 'ZYX');
rpy2 = quat2eul([attposmocap.q_0_ attposmocap.q_1_ attposmocap.q_2_, attposmocap.q_3_], 'ZYX');

figure
subplot(1,4,1)
plot(t1, rpy1(:,1))
hold on
plot(t2, rpy(:,1))
plot(t3, rpy2(:,1))
title('yaw setpoint')
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
title('roll setpoint')
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
title('roll rate')
legend('setpoint', 'measured')

subplot(1,4,2)
plot(t, vehicleratessetpoints.pitch)
hold on
plot(t2, controlstate.pitch_rate)
title('pitch rate')
legend('setpoint', 'measured')

subplot(1,4,3)
plot(t, vehicleratessetpoints.yaw)
hold on
plot(t2, controlstate.yaw_rate)
title('yaw rate')
legend('setpoint', 'measured')

subplot(1,4,4)
plot(t, vehicleratessetpoints.thrust)
title('thrust')

%% Plot actual pitch v.s. pitch setpoint v.s. pitch rate
filename = find_file('.', '.*vehicle_rates_setpoint.*')
vehicleratessetpoints = readtable(filename);
filename = find_file('.', '.*control_state.*')
controlstate = readtable(filename);
filename = find_file('.', '.*vehicle_attitude_setpoint.*')
vehicleattitudesetpoints = readtable(filename);
filename = find_file('.', '.*att_pos_mocap.*')
attposmocap = readtable(filename);

t1 = vehicleattitudesetpoints.timestamp;
t2 = controlstate.timestamp;
t3 = attposmocap.timestamp;
t4 = vehicleratessetpoints.timestamp;

rpy1 = quat2eul([vehicleattitudesetpoints.q_d_0_ vehicleattitudesetpoints.q_d_1_ vehicleattitudesetpoints.q_d_2_, vehicleattitudesetpoints.q_d_3_], 'ZYX'); 
rpy2 = quat2eul([controlstate.q_0_ controlstate.q_1_ controlstate.q_2_, controlstate.q_3_], 'ZYX');
rpy3 = quat2eul([attposmocap.q_0_ attposmocap.q_1_ attposmocap.q_2_, attposmocap.q_3_], 'ZYX');

figure
plot(t1, rpy1(:,2))
hold on
plot(t2, rpy2(:,2))
plot(t3, rpy3(:,2))
plot(t4, vehicleratessetpoints.pitch)
plot(t2, controlstate.pitch_rate)
legend('pitch setpoint', 'estimated pitch', 'mocap pitch', 'pitch rate setpoint', 'estimated pitch rate')

%% Plot actual yaw v.s. setpoint v.s. rate
filename = find_file('.', '.*vehicle_rates_setpoint.*')
vehicleratessetpoints = readtable(filename);
filename = find_file('.', '.*control_state.*')
controlstate = readtable(filename);
filename = find_file('.', '.*vehicle_attitude_setpoint.*')
vehicleattitudesetpoints = readtable(filename);
filename = find_file('.', '.*att_pos_mocap.*')
attposmocap = readtable(filename);

t1 = vehicleattitudesetpoints.timestamp;
t2 = controlstate.timestamp;
t3 = attposmocap.timestamp;
t4 = vehicleratessetpoints.timestamp;

rpy1 = quat2eul([vehicleattitudesetpoints.q_d_0_ vehicleattitudesetpoints.q_d_1_ vehicleattitudesetpoints.q_d_2_, vehicleattitudesetpoints.q_d_3_], 'ZYX'); 
rpy2 = quat2eul([controlstate.q_0_ controlstate.q_1_ controlstate.q_2_, controlstate.q_3_], 'ZYX');
rpy3 = quat2eul([attposmocap.q_0_ attposmocap.q_1_ attposmocap.q_2_, attposmocap.q_3_], 'ZYX');

figure
plot(t1, rpy1(:,1))
hold on
plot(t2, rpy2(:,1))
plot(t3, rpy3(:,1))
plot(t4, vehicleratessetpoints.yaw)
plot(t2, controlstate.yaw_rate)
legend('yaw setpoint', 'estimated yaw', 'mocap yaw', 'yaw rate setpoint', 'estimated yaw rate')
%% Plot actual roll v.s. setpoint v.s. rate
filename = find_file('.', '.*vehicle_rates_setpoint.*')
vehicleratessetpoints = readtable(filename);
filename = find_file('.', '.*control_state.*')
controlstate = readtable(filename);
filename = find_file('.', '.*vehicle_attitude_setpoint.*')
vehicleattitudesetpoints = readtable(filename);
filename = find_file('.', '.*att_pos_mocap.*')
attposmocap = readtable(filename);

t1 = vehicleattitudesetpoints.timestamp;
t2 = controlstate.timestamp;
t3 = attposmocap.timestamp;
t4 = vehicleratessetpoints.timestamp;

rpy1 = quat2eul([vehicleattitudesetpoints.q_d_0_ vehicleattitudesetpoints.q_d_1_ vehicleattitudesetpoints.q_d_2_, vehicleattitudesetpoints.q_d_3_], 'ZYX'); 
rpy2 = quat2eul([controlstate.q_0_ controlstate.q_1_ controlstate.q_2_, controlstate.q_3_], 'ZYX');
rpy3 = quat2eul([attposmocap.q_0_ attposmocap.q_1_ attposmocap.q_2_, attposmocap.q_3_], 'ZYX');

figure
plot(t1, rpy1(:,3))
hold on
plot(t2, rpy2(:,3))
plot(t3, rpy3(:,3))
plot(t4, vehicleratessetpoints.roll)
plot(t2, controlstate.roll_rate)
legend('roll setpoint', 'estimated roll', 'mocap roll', 'roll rate setpoint', 'estimated roll rate')




