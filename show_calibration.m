% show_calibration.m — Print all sensor calibration information
% Run this script to inspect raw sensor data from calibration and task files.
clear; close all; clc;

fprintf('============================================================\n');
fprintf('  SENSOR CALIBRATION SUMMARY\n');
fprintf('============================================================\n\n');

%% =====================================================================
%  1. GYROSCOPE CALIBRATION (from calib1_rotate.mat)
%% =====================================================================
fprintf('--- GYROSCOPE ---\n');
d = load('calib1_rotate.mat'); o = d.out;
gyro_all = squeeze(o.Sensor_GYRO.signals.values)';
N = size(gyro_all,1);
t_vec = (0:N-1)/200;

% Static period: first 2 seconds (robot should be still)
static_end = min(400, N);
gyro_static = gyro_all(1:static_end, :);

fprintf('  Gyro shape: [%d x %d]\n', size(gyro_all));
fprintf('  Static bias (first 2s, %d samples):\n', static_end);
for c = 1:3
    fprintf('    Col %d: mean=%+.6f rad/s  std=%.6f rad/s\n', c, ...
        mean(gyro_static(:,c)), std(gyro_static(:,c)));
end

% Determine yaw axis by looking at which column has largest range during rotation
fprintf('  Full-run stats (rotation data):\n');
for c = 1:3
    fprintf('    Col %d: min=%+.4f  max=%+.4f  range=%.4f rad/s\n', c, ...
        min(gyro_all(:,c)), max(gyro_all(:,c)), range(gyro_all(:,c)));
end

% Integrate each column to find yaw axis
gt_quat = o.GT_rotation.signals.values;
gt_yaw = zeros(N,1);
for i = 1:N
    W_=gt_quat(i,1); X_=gt_quat(i,2); Y_=gt_quat(i,3); Z_=gt_quat(i,4);
    gt_yaw(i) = atan2(2*(W_*Z_+X_*Y_), 1-2*(Y_^2+Z_^2));
end
gt_yaw_unwrap = unwrap(gt_yaw);
gt_total_rotation = gt_yaw_unwrap(end) - gt_yaw_unwrap(1);
fprintf('  GT total rotation: %.1f deg (%.1f turns)\n', ...
    rad2deg(gt_total_rotation), gt_total_rotation/(2*pi));

fprintf('  Integrated heading per column (positive vs negative sign):\n');
dt = 1/200;
for c = 1:3
    for sign = [+1, -1]
        int_yaw = cumsum(sign * (gyro_all(:,c) - mean(gyro_static(:,c)))) * dt;
        corr_val = corrcoef(int_yaw, gt_yaw_unwrap);
        fprintf('    Col %d, sign=%+d: total=%.1f deg  corr_with_GT=%.4f\n', ...
            c, sign, rad2deg(int_yaw(end)), corr_val(1,2));
    end
end

%% =====================================================================
%  2. ACCELEROMETER CALIBRATION (from calib2_straight.mat)
%% =====================================================================
fprintf('\n--- ACCELEROMETER ---\n');
d2 = load('calib2_straight.mat'); o2 = d2.out;
acc_all = squeeze(o2.Sensor_ACCEL.signals.values)';
N2 = size(acc_all, 1);

% Static period (first 2s)
static_end2 = min(400, N2);
acc_static = acc_all(1:static_end2, :);
fprintf('  Accel shape: [%d x %d]\n', size(acc_all));
fprintf('  Static bias (first 2s):\n');
for c = 1:3
    fprintf('    Col %d: mean=%+.4f m/s²  std=%.4f m/s²\n', c, ...
        mean(acc_static(:,c)), std(acc_static(:,c)));
end

% Identify gravity axis (should be ~9.81 or ~10)
fprintf('  Gravity axis identification:\n');
for c = 1:3
    if abs(mean(acc_static(:,c))) > 5
        fprintf('    Col %d: GRAVITY AXIS (mean=%.2f m/s²)\n', c, mean(acc_static(:,c)));
    end
end

% Full-run stats for forward/lateral axes
gt_pos2 = o2.GT_position.signals.values;
gt_vel = diff(gt_pos2) * 200;  % numerical velocity from GT
fprintf('  GT position range: X=[%.3f, %.3f]m  Y=[%.3f, %.3f]m\n', ...
    min(gt_pos2(:,1)), max(gt_pos2(:,1)), min(gt_pos2(:,2)), max(gt_pos2(:,2)));

fprintf('  Accel full-run stats:\n');
for c = 1:3
    fprintf('    Col %d: mean=%+.4f  std=%.4f  min=%+.4f  max=%+.4f m/s²\n', c, ...
        mean(acc_all(:,c)), std(acc_all(:,c)), min(acc_all(:,c)), max(acc_all(:,c)));
end

% LP Accelerometer
lp_acc_raw = o2.Sensor_LP_ACCEL.signals.values;
fprintf('\n  LP Accelerometer (calib2):\n');
sz = size(lp_acc_raw);
fprintf('    Raw shape: [%s]\n', num2str(sz));
% Handle various shapes (may be Nx3, 1x3xN, etc.)
if ndims(lp_acc_raw) == 3
    lp_acc_all = squeeze(lp_acc_raw)';
elseif size(lp_acc_raw,1) > 1
    lp_acc_all = lp_acc_raw;
else
    lp_acc_all = lp_acc_raw;  % single row
end
fprintf('    Processed shape: [%d x %d]\n', size(lp_acc_all,1), size(lp_acc_all,2));
if size(lp_acc_all,1) > 10 && size(lp_acc_all,2) <= 6
    for c = 1:size(lp_acc_all,2)
        fprintf('    Col %d: mean=%+.4f  std=%.4f  range=[%.4f, %.4f]\n', c, ...
            mean(lp_acc_all(:,c)), std(lp_acc_all(:,c)), min(lp_acc_all(:,c)), max(lp_acc_all(:,c)));
    end
else
    fprintf('    (unexpected shape, first 5 values: %s)\n', num2str(lp_acc_raw(1:min(5,numel(lp_acc_raw)))));
end

%% =====================================================================
%  3. ToF SENSOR CALIBRATION
%% =====================================================================
fprintf('\n--- ToF SENSORS ---\n');
% Use calib2_straight for straight-line driving
tof1_all = o2.Sensor_ToF1.signals.values;
tof2_all = o2.Sensor_ToF2.signals.values;
tof3_all = o2.Sensor_ToF3.signals.values;

tof_all_c = {tof1_all, tof2_all, tof3_all};
tof_names = {'ToF1', 'ToF2', 'ToF3'};

for s = 1:3
    fprintf('  %s:\n', tof_names{s});
    td = tof_all_c{s};
    fprintf('    Shape: [%d x %d]\n', size(td));

    valid = td(:,4) == 0 & td(:,1) >= 0.05 & td(:,1) <= 2.5 & ~isnan(td(:,1));
    n_valid = sum(valid);

    % Count unique readings (actual updates vs ZOH repeats)
    valid_readings = td(valid, 1);
    if ~isempty(valid_readings)
        changes = [true; diff(valid_readings) ~= 0];
        n_unique = sum(changes);
        fprintf('    Valid: %d/%d (%.0f%%)  |  Unique: %d (actual rate ~%.0f Hz)\n', ...
            n_valid, size(td,1), 100*n_valid/size(td,1), n_unique, ...
            n_unique / (size(td,1)/200));
        fprintf('    Range: [%.3f, %.3f] m  mean=%.3f m  std=%.3f m\n', ...
            min(valid_readings), max(valid_readings), mean(valid_readings), std(valid_readings));
    else
        fprintf('    No valid readings!\n');
    end

    % Signal strength stats
    valid_signal = td(valid, 3);
    if ~isempty(valid_signal)
        fprintf('    Signal: min=%.0f  max=%.0f  mean=%.0f  median=%.0f\n', ...
            min(valid_signal), max(valid_signal), mean(valid_signal), median(valid_signal));
    end

    % Status code breakdown
    status_codes = unique(td(:,4));
    fprintf('    Status codes: ');
    for sc = status_codes'
        fprintf('%d(%d) ', sc, sum(td(:,4)==sc));
    end
    fprintf('\n');
end

% Determine sensor orientations using GT from calib2
fprintf('\n  ToF orientation detection (calib2_straight, straight-line drive):\n');
gt_quat2 = o2.GT_rotation.signals.values;
gt_yaw2 = zeros(N2,1);
for i = 1:N2
    W_=gt_quat2(i,1); X_=gt_quat2(i,2); Y_=gt_quat2(i,3); Z_=gt_quat2(i,4);
    gt_yaw2(i) = atan2(2*(W_*Z_+X_*Y_), 1-2*(Y_^2+Z_^2));
end

arena.x_min=-1.244; arena.x_max=1.244; arena.y_min=-1.244; arena.y_max=1.244;
dir_names = {'Forward(0)', 'Left(+pi/2)', 'Backward(pi)', 'Right(-pi/2)'};
dir_angles = [0, pi/2, pi, -pi/2];

% Check first valid readings against expected wall distances
fprintf('  First valid readings vs expected wall distances from GT start:\n');
for s = 1:3
    td = tof_all_c{s};
    first_valid = NaN;
    for k = 1:size(td,1)
        if td(k,4)==0 && td(k,1)>=0.05 && td(k,1)<=2.5 && ~isnan(td(k,1))
            first_valid = td(k,1);
            break;
        end
    end
    if isnan(first_valid); continue; end

    fprintf('  %s first reading = %.3f m  →  Best match: ', tof_names{s}, first_valid);
    best_dir = ''; best_err = Inf;
    for di = 1:4
        ra = gt_yaw2(1) + dir_angles(di);
        rdx = cos(ra); rdy = sin(ra);
        dists = [];
        px = gt_pos2(1,1); py = gt_pos2(1,2);
        if abs(rdx) > 1e-9
            t = (arena.x_max - px)/rdx;
            if t>0 && py+t*rdy >= arena.y_min && py+t*rdy <= arena.y_max; dists(end+1)=t; end
            t = (arena.x_min - px)/rdx;
            if t>0 && py+t*rdy >= arena.y_min && py+t*rdy <= arena.y_max; dists(end+1)=t; end
        end
        if abs(rdy) > 1e-9
            t = (arena.y_max - py)/rdy;
            if t>0 && px+t*rdx >= arena.x_min && px+t*rdx <= arena.x_max; dists(end+1)=t; end
            t = (arena.y_min - py)/rdy;
            if t>0 && px+t*rdx >= arena.x_min && px+t*rdx <= arena.x_max; dists(end+1)=t; end
        end
        if ~isempty(dists)
            expected = min(dists);
            err = abs(first_valid - expected);
            if err < best_err
                best_err = err; best_dir = dir_names{di};
            end
        end
    end
    fprintf('%s (error=%.3fm)\n', best_dir, best_err);
end

%% =====================================================================
%  4. MAGNETOMETER CALIBRATION
%% =====================================================================
fprintf('\n--- MAGNETOMETER ---\n');
mag_all = squeeze(o.Sensor_MAG.signals.values)';  % from calib1_rotate (rotation data)
N_mag = size(mag_all, 1);
fprintf('  Mag shape: [%d x %d]\n', size(mag_all));

% Check for valid data
valid_mag = ~any(isnan(mag_all), 2);
n_valid_mag = sum(valid_mag);
fprintf('  Valid readings: %d/%d (%.0f%%)\n', n_valid_mag, N_mag, 100*n_valid_mag/N_mag);

% Unique reading count (check ZOH)
if n_valid_mag > 0
    mag_valid = mag_all(valid_mag, :);
    mag_changes = [true; any(diff(mag_valid) ~= 0, 2)];
    n_unique_mag = sum(mag_changes);
    fprintf('  Unique readings: %d (actual rate ~%.0f Hz)\n', ...
        n_unique_mag, n_unique_mag / (N_mag/200));
end

for c = 1:min(3, size(mag_all,2))
    vals = mag_all(valid_mag, c);
    fprintf('  Col %d: min=%.2e  max=%.2e  mean=%.2e  range=%.2e T\n', c, ...
        min(vals), max(vals), mean(vals), max(vals)-min(vals));
end

% Ellipse fit for hard/soft iron
if n_valid_mag > 10
    Mx = mag_valid(:,1); My = mag_valid(:,2);
    hi_x = (max(Mx)+min(Mx))/2;
    hi_y = (max(My)+min(My))/2;
    si_x = (max(Mx)-min(Mx))/2;
    si_y = (max(My)-min(My))/2;
    fprintf('  Hard-iron offset: [%.6e, %.6e] T\n', hi_x, hi_y);
    fprintf('  Soft-iron scale:  [%.6e, %.6e] T\n', si_x, si_y);

    % Calibrated heading
    Mx_cal = (Mx - hi_x) / si_x;
    My_cal = (My - hi_y) / si_y;
    mag_heading = atan2(My_cal, Mx_cal);

    % Compare with GT heading
    gt_yaw_mag = gt_yaw(valid_mag);
    heading_diff = wrapToPi(mag_heading - gt_yaw_mag);
    mag_offset = median(heading_diff);
    mag_heading_corrected = wrapToPi(mag_heading - mag_offset);
    heading_err = wrapToPi(mag_heading_corrected - gt_yaw_mag);

    fprintf('  Yaw offset (median): %.4f rad (%.1f deg)\n', mag_offset, rad2deg(mag_offset));
    fprintf('  Corrected heading error: mean=%.1f deg  std=%.1f deg\n', ...
        rad2deg(mean(heading_err)), rad2deg(std(heading_err)));
    fprintf('  Verdict: ');
    if rad2deg(std(heading_err)) < 10
        fprintf('USABLE (std < 10 deg)\n');
    elseif rad2deg(std(heading_err)) < 30
        fprintf('MARGINAL (std %.0f deg)\n', rad2deg(std(heading_err)));
    else
        fprintf('UNUSABLE (std %.0f deg — too noisy)\n', rad2deg(std(heading_err)));
    end
end

%% =====================================================================
%  5. TEMPERATURE SENSOR
%% =====================================================================
fprintf('\n--- TEMPERATURE ---\n');
temp_raw = o.Sensor_Temp.signals.values;
sz_t = size(temp_raw);
fprintf('  Raw shape: [%s]\n', num2str(sz_t));
if ndims(temp_raw) == 3
    temp_all_r = squeeze(temp_raw)';
else
    temp_all_r = temp_raw;
end
fprintf('  Processed shape: [%d x %d]\n', size(temp_all_r,1), size(temp_all_r,2));
if size(temp_all_r,1) > 1
    for c = 1:size(temp_all_r,2)
        fprintf('  Col %d: min=%.2f  max=%.2f  mean=%.2f\n', c, ...
            min(temp_all_r(:,c)), max(temp_all_r(:,c)), mean(temp_all_r(:,c)));
    end
else
    fprintf('  Values: %s\n', num2str(temp_all_r(1,1:min(5,end))));
end

%% =====================================================================
%  6. LP ACCELEROMETER (calib1_rotate)
%% =====================================================================
fprintf('\n--- LP ACCELEROMETER (calib1_rotate) ---\n');
lp_raw_r = o.Sensor_LP_ACCEL.signals.values;
sz_lp = size(lp_raw_r);
fprintf('  Raw shape: [%s]\n', num2str(sz_lp));
if ndims(lp_raw_r) == 3
    lp_all_r = squeeze(lp_raw_r)';
else
    lp_all_r = lp_raw_r;
end
fprintf('  Processed shape: [%d x %d]\n', size(lp_all_r,1), size(lp_all_r,2));
if size(lp_all_r,1) > 1 && size(lp_all_r,2) <= 6
    lp_changes = [true; any(diff(lp_all_r) ~= 0, 2)];
    fprintf('  Unique: %d (actual rate ~%.0f Hz)\n', sum(lp_changes), sum(lp_changes)/(size(lp_all_r,1)/200));
    for c = 1:size(lp_all_r,2)
        fprintf('  Col %d: min=%.4f  max=%.4f  mean=%.4f  std=%.4f\n', c, ...
            min(lp_all_r(:,c)), max(lp_all_r(:,c)), mean(lp_all_r(:,c)), std(lp_all_r(:,c)));
    end
end

%% =====================================================================
%  7. CROSS-DATASET CONSISTENCY CHECK
%% =====================================================================
fprintf('\n--- CROSS-DATASET CONSISTENCY ---\n');
datasets = {'task1_1 1.mat','task1_2 1.mat','task1_3.mat','task2_1 1.mat','task2_2 1.mat','task2_3 1.mat'};
fprintf('  %-18s  %6s  %10s  %10s  %12s  %12s\n', ...
    'Dataset', 'N', 'GT_x0', 'GT_y0', 'GT_yaw0(deg)', 'Duration(s)');
for di = 1:numel(datasets)
    dd = load(datasets{di}); oo = dd.out;
    acc_d = squeeze(oo.Sensor_ACCEL.signals.values)';
    Nd = size(acc_d,1);
    gtp = oo.GT_position.signals.values;
    gtq = oo.GT_rotation.signals.values;
    W_=gtq(1,1); X_=gtq(1,2); Y_=gtq(1,3); Z_=gtq(1,4);
    yaw0 = atan2(2*(W_*Z_+X_*Y_), 1-2*(Y_^2+Z_^2));
    fprintf('  %-18s  %6d  %10.3f  %10.3f  %12.1f  %12.1f\n', ...
        datasets{di}, Nd, gtp(1,1), gtp(1,2), rad2deg(yaw0), Nd/200);
end

% Also check task2_4 if it exists
if exist('task2_4.mat', 'file')
    fprintf('\n  task2_4.mat found (extra dataset):\n');
    dd = load('task2_4.mat'); oo = dd.out;
    acc_d = squeeze(oo.Sensor_ACCEL.signals.values)';
    Nd = size(acc_d,1);
    gtp = oo.GT_position.signals.values;
    gtq = oo.GT_rotation.signals.values;
    W_=gtq(1,1); X_=gtq(1,2); Y_=gtq(1,3); Z_=gtq(1,4);
    yaw0 = atan2(2*(W_*Z_+X_*Y_), 1-2*(Y_^2+Z_^2));
    fprintf('    N=%d  GT_start=(%.3f, %.3f)  yaw0=%.1f deg  duration=%.1fs\n', ...
        Nd, gtp(1,1), gtp(1,2), rad2deg(yaw0), Nd/200);
end

fprintf('\n============================================================\n');
fprintf('  CURRENT myEKF CALIBRATION VALUES\n');
fprintf('============================================================\n');
fprintf('  GYRO:   col=1  sign=+1  bias=+0.001883 rad/s\n');
fprintf('  ACCEL:  fwd=col3 sign=+1 bias=-0.3962  lat=col2 sign=+1 bias=+0.0278\n');
fprintf('  ToF:    1->Left(+pi/2)  2->Backward(+pi)  3->Right(-pi/2)\n');
fprintf('  MAG:    DISABLED (78 deg std)\n');
fprintf('  R_tof=0.02  R_accel=%.4f  R_gyro=%.6f  R_mag=%.4f\n', 1.02^2, 0.005^2, 1.8646);
fprintf('  ZUPT_R=0.001  OMEGA_STILL_THRESH=0.08\n');
fprintf('============================================================\n');
