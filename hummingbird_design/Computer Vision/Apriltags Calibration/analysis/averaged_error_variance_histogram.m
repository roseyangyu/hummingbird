%% Variance Plots (Averaged)

% Positional Variance
for i = 1:length(ts_estimate1_poses)
    ts_estimate_poses(:,:,1) = ts_estimate1_poses(:,:,i);
    ts_estimate_poses(:,:,2) = ts_estimate2_poses(:,:,i);
    ts_estimate_poses(:,:,3) = ts_estimate3_poses(:,:,i);
    ts_estimate_poses(:,:,4) = ts_estimate4_poses(:,:,i);
    ts_average_poses(:,:,i) = pose_mean(ts_estimate_poses);
end

ts_average_position = squeeze(ts_average_poses(1:3,4,:));
var_ts_average_x = var(ts_average_position(1,:)' - ground_truth(duration,1));
var_ts_average_y = var(ts_average_position(2,:)' - ground_truth(duration,2));
var_ts_average_z = var(ts_average_position(3,:)' - ground_truth(duration,3));

% Sample Histogram
figure;
subplot(2,2,1);
hold on;
histogram(vecnorm(ts_average_position(1:3, duration)' - ground_truth(duration,1:3),2,2))
title(strcat('Total Distance to Tailsitter vs TS Esimates Averaged (STD: ', num2str(sqrt(var_ts_average_x)), ')'));
xlabel('Total Positional Distance to Tailsitter')
ylabel('Frequency')
grid minor;

subplot(2,2,2);
hold on;
histogram(ts_average_position(1, duration)' - ground_truth(duration,1))
title(strcat('X Distance to Tailsitter vs TS Esimates Averaged (STD: ', num2str(sqrt(var_ts_average_x)), ')'));
xlabel('X Positional Distance to Tailsitter')
ylabel('Frequency')
grid minor;

subplot(2,2,3);
hold on;
histogram(ts_average_position(2, duration)' - ground_truth(duration,2))
title(strcat('Y Distance to Tailsitter vs TS Esimates Averaged (STD: ', num2str(sqrt(var_ts_average_y)), ')'));
xlabel('Y Positional Distance to Tailsitter')
ylabel('Frequency')
grid minor;

subplot(2,2,4);
hold on;
histogram(ts_average_position(3, duration)' - ground_truth(duration,3))
title(strcat('Z Distance to Tailsitter vs TS Esimates Averaged (STD: ', num2str(sqrt(var_ts_average_z)), ')'));
xlabel('Z Positional Distance to Tailsitter')
ylabel('Frequency')
grid minor;
