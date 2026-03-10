%==========================================================================
% calibrate_rotate.m
% Analyses Calibration Data 2 (calib1_rotate.mat):
%   Robot completes 5 full CCW rotations in place.
%
% OUTPUTS:
%   - Which gyro column is the yaw axis
%   - Correct GYRO_Z_SIGN (+1 or -1)
%   - Gyro bias (rad/s)
%   - Gyro scale factor (via linear regression vs GT)
%   - Noise std for R_gyro
%
% USAGE: Set CAL_FILE, run, copy printed block into myEKF.m
%==========================================================================
clear; close all; clc;

CAL_FILE = 'calib1_rotate.mat';   % <-- set to your rotation .mat filename

%--------------------------------------------------------------------------
% Load
%--------------------------------------------------------------------------
data = load(CAL_FILE);
f    = fieldnames(data);
if isfield(data,'out');  out = data.out;
elseif length(f)==1;     out = data.(f{1});
else
    for i=1:length(f); fprintf('%d: %s\n',i,f{i}); end
    out = data.(f{input('Enter field number: ')});
end

%--------------------------------------------------------------------------
% Extract sensors — same pattern as myEKF.m
%--------------------------------------------------------------------------
gyro      = squeeze(out.Sensor_GYRO.signals.values)';   % [N x 3]
sens_time = squeeze(out.Sensor_Time.signals.values);
sens_time = sens_time(:);
gt_rot    = out.GT_rotation.signals.values;             % [N x 4] w,x,y,z
gt_pos    = out.GT_position.signals.values;             % [N x 3]

N    = length(sens_time);
t    = sens_time - sens_time(1);
dt   = [1/104; diff(t)];
dt(dt<=0 | dt>0.5) = 1/104;

fprintf('Loaded: %d samples  (%.1f s)\n', N, t(end));

%--------------------------------------------------------------------------
% GT yaw — try all 4 conventions, pick smallest range (rotation cal should
% show ~1800 deg = 5 full turns, all conventions will show this but signs differ)
%--------------------------------------------------------------------------
cands = zeros(N,4);
for i = 1:N
    w=gt_rot(i,1); qx=gt_rot(i,2); qy=gt_rot(i,3); qz=gt_rot(i,4);
    cands(i,1) = atan2( 2*(w*qz+qx*qy), 1-2*(qy^2+qz^2));
    cands(i,2) = atan2( 2*(w*qx+qy*qz), 1-2*(qx^2+qz^2));
    cands(i,3) = atan2( 2*(w*qy+qx*qz), 1-2*(qx^2+qy^2));  % confirmed correct
    cands(i,4) = atan2(-2*(w*qz+qx*qy), 1-2*(qy^2+qz^2));
end

% Unwrap all conventions — for rotation cal, correct one gives ~10*pi total
cands_uw = unwrap(cands);
cands_rel = cands_uw - cands_uw(1,:);

% Use convention 3 (confirmed from task data) — but print all for verification
QUAT_CONV = 3;
gt_yaw_rel = cands_rel(:, QUAT_CONV);
cname = {'standard-Z','roll-axis','pitch-axis (conv3)','neg-Z'};
fprintf('\nGT yaw (unwrapped) at end by convention:\n');
for c=1:4
    fprintf('  conv %d %-22s: %+8.1f deg\n', c, cname{c}, rad2deg(cands_rel(end,c)));
end
fprintf('  → Using conv %d.  5 full turns = ±1800 deg.\n', QUAT_CONV);
fprintf('    Sign tells rotation direction (CCW = positive in standard math)\n\n');

%--------------------------------------------------------------------------
% Auto-detect stationary window at start
% Scan 0.5s windows — stop when any gyro column std exceeds threshold
%--------------------------------------------------------------------------
win   = round(0.5 * 104);
thresh = 0.02;   % rad/s
stat_end = win;
for i = 1:win:N-win
    seg_std = std(double(gyro(i:i+win-1, :)));
    if max(seg_std) > thresh
        break
    end
    stat_end = i + win - 1;
end
stat_end = max(stat_end, 20);
fprintf('Stationary window: samples 1–%d  (%.2f s)\n', stat_end, t(stat_end));

%--------------------------------------------------------------------------
% Bias for all 3 columns from stationary window
%--------------------------------------------------------------------------
bias_all = mean(double(gyro(1:stat_end, :)));
std_all  = std( double(gyro(1:stat_end, :)));

fprintf('\n--- Gyro bias from stationary window ---\n');
fprintf('%-8s  %12s  %10s\n','Column','bias (rad/s)','std (rad/s)');
for c=1:3
    fprintf('  col %d     %+.6f    %.6f\n', c, bias_all(c), std_all(c));
end

%--------------------------------------------------------------------------
% Test all 3 columns: integrate and compare to GT
% The correct yaw column will match GT total rotation
%--------------------------------------------------------------------------
gt_rate = [0; diff(gt_yaw_rel)] ./ dt;

fprintf('\n--- Column analysis vs GT ---\n');
fprintf('%-8s  %10s  %10s  %8s  %10s  %s\n', ...
    'Column','integrated','GT_total','scale','corr(r)','sign_ok');

results = struct();
for c = 1:3
    gz = double(gyro(:,c)) - bias_all(c);
    theta_int = cumsum(gz .* dt);
    theta_int = theta_int - theta_int(1);

    % Scale via linear regression on moving samples
    moving = abs(gz) > 0.05;
    if sum(moving) > 50
        A = theta_int(moving);
        b = gt_yaw_rel(moving);
        scale = (A'*A) \ (A'*b);
    else
        scale = gt_yaw_rel(end) / (theta_int(end) + 1e-9);
    end

    % Correlation with GT rate
    C = corrcoef(gz, gt_rate);
    r = C(1,2);

    sign_ok = sign(theta_int(end)) == sign(gt_yaw_rel(end));

    results(c).theta_int = theta_int;
    results(c).scale     = scale;
    results(c).r         = r;
    results(c).sign_ok   = sign_ok;
    results(c).bias      = bias_all(c);
    results(c).std       = std_all(c);

    if sign_ok; sign_str = 'YES'; else; sign_str = 'NO - flip GYRO_Z_SIGN'; end
    fprintf('  col %d    %+8.1f deg  %+8.1f deg  %6.3f   %+.3f      %s\n', ...
        c, rad2deg(theta_int(end)), rad2deg(gt_yaw_rel(end)), ...
        scale, r, sign_str);
end

%--------------------------------------------------------------------------
% Identify best column: highest |r| with GT yaw rate
%--------------------------------------------------------------------------
r_vals  = abs([results(1).r, results(2).r, results(3).r]);
[~, best_col] = max(r_vals);
best_sign = sign(results(best_col).r);   % positive r = sign +1
if best_sign >= 0; best_sign_val = 1; else; best_sign_val = -1; end

best_bias  = results(best_col).bias;
best_scale = results(best_col).scale * best_sign_val;  % absorb sign into scale
best_std   = results(best_col).std;

% If scale is very close to 1, no correction needed
scale_needed = abs(best_scale - 1.0) > 0.05;

%--------------------------------------------------------------------------
% Print copy-paste block
%--------------------------------------------------------------------------
fprintf('\n==========================================================\n');
fprintf(' RESULTS — COPY INTO myEKF.m\n');
fprintf('==========================================================\n');
fprintf('  GYRO_Z_IDX  = %d;\n', best_col);
fprintf('  GYRO_Z_SIGN = %d;\n', best_sign_val);
if scale_needed
    fprintf('  GYRO_SCALE  = %.4f;  %% >5%% error — apply correction\n', abs(best_scale));
else
    fprintf('  GYRO_SCALE  = 1.0;   %% within 5%% — no correction needed\n');
end
fprintf('\n  %% Bias is auto-estimated at startup in myEKF.m (first 200 samples)\n');
fprintf('  %% Measured bias for reference: %.6f rad/s\n', best_bias);
fprintf('  %% R_gyro lower bound from std:  (%.4f)^2 = %.2e\n', best_std, best_std^2);
fprintf('==========================================================\n\n');

%--------------------------------------------------------------------------
% Plots
%--------------------------------------------------------------------------
figure('Name','Gyro Calibration — Rotation','Position',[50 50 1100 700]);

% Plot 1: All 3 integrated vs GT
subplot(2,3,1:2);
plot(t, rad2deg(gt_yaw_rel), 'b-', 'LineWidth',2, 'DisplayName','GT yaw');
hold on;
colours = {'r','m','g'};
for c=1:3
    gz_s = results(best_col).scale * (double(gyro(:,c)) - bias_all(c)) * best_sign_val;
    if c==best_col; lw=2; else; lw=0.8; end
    plot(t, rad2deg(cumsum(gz_s.*dt)), colours{c}, 'LineWidth',lw, ...
         'DisplayName', sprintf('col%d (r=%.2f)', c, results(c).r));
end
ylabel('Cumulative rotation (deg)'); xlabel('Time (s)');
legend('Location','northwest'); grid on;
title(sprintf('All columns vs GT  |  Best: col %d  sign=%+d  scale=%.4f', ...
    best_col, best_sign_val, abs(best_scale)));

% Plot 2: Best column — GT vs integrated with and without scale
subplot(2,3,3);
gz_best = best_sign_val * (double(gyro(:,best_col)) - best_bias);
theta_raw   = cumsum(gz_best .* dt);
theta_scaled = theta_raw * abs(best_scale);
plot(t, rad2deg(gt_yaw_rel), 'b', 'LineWidth',2, 'DisplayName','GT');
hold on;
plot(t, rad2deg(theta_raw),    'r--','LineWidth',1.5, 'DisplayName','Integrated (no scale)');
plot(t, rad2deg(theta_scaled), 'g-', 'LineWidth',1.5, 'DisplayName','Integrated × scale');
ylabel('deg'); xlabel('Time (s)'); legend; grid on;
title(sprintf('Col %d — scale correction', best_col));

% Plot 3: Yaw rate comparison
subplot(2,3,4:5);
plot(t, rad2deg(gt_rate), 'b', 'DisplayName','GT yaw rate');
hold on;
plot(t, rad2deg(gz_best), 'r', 'DisplayName', sprintf('Gyro col %d (bias-corrected)', best_col));
ylabel('Yaw rate (deg/s)'); xlabel('Time (s)'); legend; grid on;
title('Yaw rate — should overlap closely');

% Plot 4: Stationary window used for bias
subplot(2,3,6);
plot(t(1:stat_end), rad2deg(double(gyro(1:stat_end, best_col))), 'k');
yline(rad2deg(best_bias), 'r--', sprintf('bias=%.5f rad/s', best_bias), 'LineWidth',1.5);
xlabel('Time (s)'); ylabel('Gyro col Z (deg/s)');
title(sprintf('Stationary window (%d samples)', stat_end)); grid on;

sgtitle('Gyro Calibration — Rotation File');