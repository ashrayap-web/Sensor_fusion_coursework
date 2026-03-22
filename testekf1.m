clear; close all; clc;

% Load Data
DATA_FILE = 'task1_1 1.mat'; 
data = load(DATA_FILE);
out = data.out;

% Extract Data Streams
acc_all = squeeze(out.Sensor_ACCEL.signals.values)';  
gyro_all = squeeze(out.Sensor_GYRO.signals.values)';   
mag_all = squeeze(out.Sensor_MAG.signals.values)';    
tof1_all = out.Sensor_ToF1.signals.values;   
tof2_all = out.Sensor_ToF2.signals.values;
tof3_all = out.Sensor_ToF3.signals.values;
temp_all = out.Sensor_Temp.signals.values;
lp_acc_all = out.Sensor_LP_ACCEL.signals.values;

N = size(acc_all, 1);
X_History = zeros(8, N);

fprintf('Processing %d samples...\n', N);
for k = 1:N
    % Pass real data if within array bounds, else NaN to extrapolate
    mag = get_dat(mag_all, k);
    t1 = get_dat(tof1_all, k);
    t2 = get_dat(tof2_all, k);
    t3 = get_dat(tof3_all, k);
    tmp = get_dat(temp_all, k);
    lp = get_dat(lp_acc_all, k);
    
    [X_Est, ~] = myEKF(acc_all(k,:), gyro_all(k,:), mag, t1, t2, t3, tmp, lp);
    X_History(:, k) = X_Est;
end

% Plotting
gt = out.GT_position.signals.values;
rmse = sqrt(mean(sum((X_History([1,4],1:N)' - gt(1:N,1:2)).^2, 2)));

figure;
plot(gt(:,1), gt(:,2), 'b--', 'LineWidth', 1.5, 'DisplayName', 'Ground Truth (Blue)'); hold on;
plot(X_History(1,:), X_History(4,:), 'r', 'LineWidth', 1.2, 'DisplayName', 'EKF Estimate');
axis equal; grid on;
title(['RMSE: ', num2str(rmse), ' m']);
legend('Location','best');

function v = get_dat(arr, k)
    if k <= size(arr,1), v = arr(k,:); else, v = NaN(1, size(arr,2)); end
end