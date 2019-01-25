%% Variance Plots (Distribution)

clear var_ground_truth var_ts_estimate1 var_ts_estimate2 var_ts_estimate3 var_ts_estimate4
clear var_ts_estimate1_x var_ts_estimate2_x var_ts_estimate3_x
clear var_ts_estimate1_y var_ts_estimate2_y var_ts_estimate3_y
clear var_ts_estimate1_z var_ts_estimate2_z var_ts_estimate3_z
clear var_ts_estimate var_ts_estimate_x var_ts_estimate_y var_ts_estimate_z
clear avg_distance

% Positional Variance
bins = 50;
for i = 1:bins-1
    range = floor((i-1)*(endtime/bins))+1:floor((i-1)*endtime/bins+2*endtime/bins);
    
    var_ground_truth(i) = var(vecnorm(ground_truth(range,1:3),2,2)); % m;
    var_ts_estimate1(i) = var(vecnorm(ts_estimate1(range,1:3),2,2)); % m;
    var_ts_estimate2(i) = var(vecnorm(ts_estimate2(range,1:3),2,2)); % m;
    var_ts_estimate3(i) = var(vecnorm(ts_estimate3(range,1:3),2,2)); % m;
    var_ts_estimate4(i) = var(vecnorm(ts_estimate4(range,1:3),2,2)); % m;

    var_ts_estimate1_x(i) = var(ts_estimate1(range,1)); % m;
    var_ts_estimate2_x(i) = var(ts_estimate2(range,1)); % m;
    var_ts_estimate3_x(i) = var(ts_estimate3(range,1)); % m;
    var_ts_estimate4_x(i) = var(ts_estimate4(range,1)); % m;
    var_ts_estimate1_y(i) = var(ts_estimate1(range,2)); % m;
    var_ts_estimate2_y(i) = var(ts_estimate2(range,2)); % m;
    var_ts_estimate3_y(i) = var(ts_estimate3(range,2)); % m;
    var_ts_estimate4_y(i) = var(ts_estimate4(range,2)); % m;
    var_ts_estimate1_z(i) = var(ts_estimate1(range,3)); % m;
    var_ts_estimate2_z(i) = var(ts_estimate2(range,3)); % m;
    var_ts_estimate3_z(i) = var(ts_estimate3(range,3)); % m;
    var_ts_estimate4_z(i) = var(ts_estimate4(range,3)); % m;

    var_ts_estimate(i) = (var_ts_estimate1(i) + var_ts_estimate2(i) + var_ts_estimate3(i) + var_ts_estimate4(i)) / 4;
    var_ts_estimate_x(i) = (var_ts_estimate1_x(i) + var_ts_estimate2_x(i) + var_ts_estimate3_x(i) + var_ts_estimate4_x(i)) / 4;
    var_ts_estimate_y(i) = (var_ts_estimate1_y(i) + var_ts_estimate2_y(i) + var_ts_estimate3_y(i) + var_ts_estimate4_y(i)) / 4;
    var_ts_estimate_z(i) = (var_ts_estimate1_z(i) + var_ts_estimate2_z(i) + var_ts_estimate3_z(i) + var_ts_estimate4_z(i)) / 4;
    
    avg_distance(i) = mean(distance_ts_to_camera(range));
end

figure;
hold off;
scatter(avg_distance, sqrt(var_ts_estimate));
grid minor;
title('Average Distance vs Standard Deviation')
xlabel('Distance')
ylabel('Variance')
