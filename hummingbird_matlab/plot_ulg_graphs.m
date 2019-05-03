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