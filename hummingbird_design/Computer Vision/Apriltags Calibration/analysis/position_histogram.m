%% Variance Plots (Entire Range)

% Positional Variance
var_ground_truth = var(vecnorm(ground_truth(duration,1:3),2,2)); % m;
var_ts_estimate1 = var(vecnorm(ts_estimate1(duration,1:3),2,2)); % m;
var_ts_estimate2 = var(vecnorm(ts_estimate2(duration,1:3),2,2)); % m;
var_ts_estimate3 = var(vecnorm(ts_estimate3(duration,1:3),2,2)); % m;
var_ts_estimate4 = var(vecnorm(ts_estimate4(duration,1:3),2,2)); % m;

var_ts_estimate1_x = var(ts_estimate1(duration,1)); % m;
var_ts_estimate2_x = var(ts_estimate2(duration,1)); % m;
var_ts_estimate3_x = var(ts_estimate3(duration,1)); % m;
var_ts_estimate4_x = var(ts_estimate4(duration,1)); % m;
var_ts_estimate1_y = var(ts_estimate1(duration,2)); % m;
var_ts_estimate2_y = var(ts_estimate2(duration,2)); % m;
var_ts_estimate3_y = var(ts_estimate3(duration,2)); % m;
var_ts_estimate4_y = var(ts_estimate4(duration,2)); % m;
var_ts_estimate1_z = var(ts_estimate1(duration,3)); % m;
var_ts_estimate2_z = var(ts_estimate2(duration,3)); % m;
var_ts_estimate3_z = var(ts_estimate3(duration,3)); % m;
var_ts_estimate4_z = var(ts_estimate4(duration,3)); % m;

var_ts_estimate = (var_ts_estimate1 + var_ts_estimate2 + var_ts_estimate3 + var_ts_estimate4) / 4;
var_ts_estimate_x = (var_ts_estimate1_x + var_ts_estimate2_x + var_ts_estimate3_x + var_ts_estimate4_x) / 4;
var_ts_estimate_y = (var_ts_estimate1_y + var_ts_estimate2_y + var_ts_estimate3_y + var_ts_estimate4_y) / 4;
var_ts_estimate_z = (var_ts_estimate1_z + var_ts_estimate2_z + var_ts_estimate3_z + var_ts_estimate4_z) / 4;

window_size = 10;

% Sample Histogram
figure;
subplot(2,2,1);
hold on;
histogram(vecnorm(ground_truth(duration,1:3),2,2))
histogram(vecnorm(ts_estimate1(duration,1:3),2,2))
histogram(vecnorm(ts_estimate2(duration,1:3),2,2))
histogram(vecnorm(ts_estimate3(duration,1:3),2,2))
histogram(vecnorm(ts_estimate4(duration,1:3),2,2))
legend('Ground truth', 'TS Estimate 1', 'TS Estimate 2', 'TS Estimate 3', 'TS Estimate 4');
title(strcat('Total Distance to Tailsitter vs TS Esimates (STD: ', num2str(sqrt(var_ts_estimate)), ')'));
xlabel('Total Positional Distance to Tailsitter')
ylabel('Frequency')
grid minor;

subplot(2,2,2);
hold on;
histogram(ground_truth(duration,1))
histogram(ts_estimate1(duration,1))
histogram(ts_estimate2(duration,1))
histogram(ts_estimate3(duration,1))
histogram(ts_estimate4(duration,1))
legend('Ground truth', 'TS Estimate 1', 'TS Estimate 2', 'TS Estimate 3', 'TS Estimate 4');
title(strcat('X Distance to Tailsitter vs TS Esimates (STD: ', num2str(sqrt(var_ts_estimate_x)), ')'));
xlabel('X Positional Distance to Tailsitter')
ylabel('Frequency')
grid minor;

subplot(2,2,3);
hold on;
histogram(ground_truth(duration,2))
histogram(ts_estimate1(duration,2))
histogram(ts_estimate2(duration,2))
histogram(ts_estimate3(duration,2))
histogram(ts_estimate4(duration,2))
legend('Ground truth', 'TS Estimate 1', 'TS Estimate 2', 'TS Estimate 3', 'TS Estimate 4');
title(strcat('X Distance to Tailsitter vs TS Esimates (STD: ', num2str(sqrt(var_ts_estimate_y)), ')'));
xlabel('Y Positional Distance to Tailsitter')
ylabel('Frequency')
grid minor;

subplot(2,2,4);
hold on;
histogram(ground_truth(duration,3))
histogram(ts_estimate1(duration,3))
histogram(ts_estimate2(duration,3))
histogram(ts_estimate3(duration,3))
histogram(ts_estimate4(duration,3))
legend('Ground truth', 'TS Estimate 1', 'TS Estimate 2', 'TS Estimate 3', 'TS Estimate 4');
title(strcat('X Distance to Tailsitter vs TS Esimates (STD: ', num2str(sqrt(var_ts_estimate_z)), ')'));
xlabel('Z Positional Distance to Tailsitter')
ylabel('Frequency')
grid minor;