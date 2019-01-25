%% Distance to tags vs time

delta_ts_estimate1_ground_truth = ts_estimate1(duration, 1:3) - ground_truth(duration, 1:3);
delta_ts_estimate2_ground_truth = ts_estimate2(duration, 1:3) - ground_truth(duration, 1:3);
delta_ts_estimate3_ground_truth = ts_estimate3(duration, 1:3) - ground_truth(duration, 1:3);
delta_ts_estimate4_ground_truth = ts_estimate4(duration, 1:3) - ground_truth(duration, 1:3);
delta_ts_ground_truth_camera = ground_truth(duration, 1:3) - r200_location(duration, 1:3);

distance_ts_estimate1 = vecnorm(delta_ts_estimate1_ground_truth,2,2);
distance_ts_estimate2 = vecnorm(delta_ts_estimate2_ground_truth,2,2);
distance_ts_estimate3 = vecnorm(delta_ts_estimate3_ground_truth,2,2);
distance_ts_estimate4 = vecnorm(delta_ts_estimate4_ground_truth,2,2);
distance_ts_to_camera = vecnorm(delta_ts_ground_truth_camera,2,2);

figure;
subplot(2,2,1);
hold on;
scatter(time(duration), distance_ts_estimate1, '.');
scatter(time(duration), distance_ts_estimate2, '.');
scatter(time(duration), distance_ts_estimate3, '.');
scatter(time(duration), distance_ts_estimate4, '.');
title('Time vs Error in Tailsitter Position')
xlabel('Time');
ylabel('Position Error with ground truth');
grid minor;

subplot(2,2,2);
hold on;
scatter(time(duration), delta_ts_estimate1_ground_truth(duration,1), '.');
scatter(time(duration), delta_ts_estimate2_ground_truth(duration,1), '.');
scatter(time(duration), delta_ts_estimate3_ground_truth(duration,1), '.');
scatter(time(duration), delta_ts_estimate4_ground_truth(duration,1), '.');
title('Time vs Error in Tailsitter Position X')
xlabel('Time');
ylabel('Position Error with ground truth X');
grid minor;

subplot(2,2,3);
hold on;
scatter(time(duration), delta_ts_estimate1_ground_truth(duration,2), '.');
scatter(time(duration), delta_ts_estimate2_ground_truth(duration,2), '.');
scatter(time(duration), delta_ts_estimate3_ground_truth(duration,2), '.');
scatter(time(duration), delta_ts_estimate4_ground_truth(duration,2), '.');
title('Time vs Error in Tailsitter Position Y')
xlabel('Time');
ylabel('Position Error with ground truth Y');
grid minor;

subplot(2,2,4);
hold on;
scatter(time(duration), delta_ts_estimate1_ground_truth(duration,3), '.');
scatter(time(duration), delta_ts_estimate2_ground_truth(duration,3), '.');
scatter(time(duration), delta_ts_estimate3_ground_truth(duration,3), '.');
scatter(time(duration), delta_ts_estimate4_ground_truth(duration,3), '.');
title('Time vs Error in Tailsitter Position Z')
xlabel('Time');
ylabel('Position Error with ground truth Z');
grid minor;