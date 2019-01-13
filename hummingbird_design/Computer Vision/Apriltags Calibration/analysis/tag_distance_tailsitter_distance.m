%% Distance to tags vs distance to camera

figure;
subplot(2,2,1);
hold on;
scatter(distance_ts_to_camera, distance_ts_estimate1, '.');
scatter(distance_ts_to_camera, distance_ts_estimate2, '.');
scatter(distance_ts_to_camera, distance_ts_estimate3, '.');
scatter(distance_ts_to_camera, distance_ts_estimate4, '.');
title('Total Distance Between Tailsitter and Camera vs Error in Tailsitter Position')
xlabel('Distance Between Tailsitter and Camera');
ylabel('Position Error with ground truth');
grid minor;

subplot(2,2,2);
hold on;
scatter(delta_ts_ground_truth_camera(duration,1), delta_ts_estimate1_ground_truth(duration,1), '.');
scatter(delta_ts_ground_truth_camera(duration,1), delta_ts_estimate2_ground_truth(duration,1), '.');
scatter(delta_ts_ground_truth_camera(duration,1), delta_ts_estimate3_ground_truth(duration,1), '.');
scatter(delta_ts_ground_truth_camera(duration,1), delta_ts_estimate4_ground_truth(duration,1), '.');
title('Total Distance Between Tailsitter X and Camera vs Error in Tailsitter Position X')
xlabel('Distance Between Tailsitter and Camera X');
ylabel('Position Error with ground truth X');
grid minor;

subplot(2,2,3);
hold on;
scatter(delta_ts_ground_truth_camera(:,2), delta_ts_estimate1_ground_truth(duration,2), '.');
scatter(delta_ts_ground_truth_camera(:,2), delta_ts_estimate2_ground_truth(duration,2), '.');
scatter(delta_ts_ground_truth_camera(:,2), delta_ts_estimate3_ground_truth(duration,2), '.');
scatter(delta_ts_ground_truth_camera(:,2), delta_ts_estimate4_ground_truth(duration,2), '.');
title('Total Distance Between Tailsitter Y and Camera vs Error in Tailsitter Position Y')
xlabel('Distance Between Tailsitter and Camera Y');
ylabel('Position Error with ground truth Y');
grid minor;

subplot(2,2,4);
hold on;
scatter(delta_ts_ground_truth_camera(duration,3), delta_ts_estimate1_ground_truth(duration,3), '.');
scatter(delta_ts_ground_truth_camera(duration,3), delta_ts_estimate2_ground_truth(duration,3), '.');
scatter(delta_ts_ground_truth_camera(duration,3), delta_ts_estimate3_ground_truth(duration,3), '.');
scatter(delta_ts_ground_truth_camera(duration,3), delta_ts_estimate4_ground_truth(duration,3), '.');
title('Total Distance Between Tailsitter Z and Camera vs Error in Tailsitter Position Z')
xlabel('Distance Between Tailsitter and Camera Z');
ylabel('Position Error with ground truth Z');
grid minor;