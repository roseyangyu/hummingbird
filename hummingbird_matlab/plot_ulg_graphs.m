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

filename = find_file('.', '.*vehicle_local_position.*')
vehiclelocalposition = readtable(filename);

filename = find_file('.', '.*att_pos_mocap.*')
attposmocap = readtable(filename);

t1 = positionsetpointtriplet.current_timestamp;
t2 = vehiclelocalposition.timestamp;
t3 = attposmocap.timestamp;

% plot z tracking
figure
subplot(1,3,1)
plot(t1, positionsetpointtriplet.current_x)
hold on
plot(t2, vehiclelocalposition.x)
plot(t3, attposmocap.x)
legend('setpoint', 'measured', 'mocap')

subplot(1,3,2)
plot(t1, positionsetpointtriplet.current_y)
hold on
plot(t2, vehiclelocalposition.y)
plot(t3, attposmocap.y)
legend('setpoint', 'measured', 'mocap')

subplot(1,3,3)
plot(t1, positionsetpointtriplet.current_z)
hold on
plot(t2, vehiclelocalposition.z)
plot(t3, attposmocap.z)
legend('setpoint', 'measured', 'mocap')

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
