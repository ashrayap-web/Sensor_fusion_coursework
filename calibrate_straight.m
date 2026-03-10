%==========================================================================
% calibrate_straight.m
% Analyses Calibration Data 1 (calib2_straight.mat):
%   - Robot stationary for ~1 minute   → accel bias, gyro bias, noise floors
%   - Robot drives straight forward/backward → accel axis mapping,
%     gravity axis identification
%
% OUTPUTS:
%   - Which accel column is gravity (do NOT use in EKF)
%   - ACCEL_X_IDX, ACCEL_X_SIGN  (body-forward)
%   - ACCEL_Y_IDX, ACCEL_Y_SIGN  (body-lateral)
%   - Accel and gyro bias per column
%   - Noise std for Q and R matrices
%
% USAGE: Set CAL_FILE, run, copy printed block into myEKF.m
%==========================================================================
clear; close all; clc;

CAL_FILE = 'calib2_straight.mat';   % <-- set to your straight .mat filename

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
accel     = squeeze(out.Sensor_ACCEL.signals.values)';
gyro      = squeeze(out.Sensor_GYRO.signals.values)';
sens_time = squeeze(out.Sensor_Time.signals.values);
sens_time = sens_time(:);
gt_pos    = out.GT_position.signals.values;
gt_rot    = out.GT_rotation.signals.values;

N  = length(sens_time);
t  = sens_time - sens_time(1);
dt = [1/104; diff(t)];
dt(dt<=0 | dt>0.5) = 1/104;

fprintf('Loaded: %d samples  (%.1f s)\n', N, t(end));

%--------------------------------------------------------------------------
% GT yaw — convention 3 (pitch-axis), confirmed correct
%--------------------------------------------------------------------------
gt_yaw = zeros(N,1);
for i = 1:N
    w=gt_rot(i,1); qx=gt_rot(i,2); qy=gt_rot(i,3); qz=gt_rot(i,4);
    gt_yaw(i) = atan2(2*(w*qy+qx*qz), 1-2*(qx^2+qy^2));
end

%--------------------------------------------------------------------------
% GT velocity (smoothed finite difference)
%--------------------------------------------------------------------------
K = ones(5,1)/5;
gt_vx = conv([0; diff(gt_pos(:,1))]./dt, K, 'same');
gt_vy = conv([0; diff(gt_pos(:,2))]./dt, K, 'same');
gt_speed = sqrt(gt_vx.^2 + gt_vy.^2);

% GT acceleration
gt_ax = conv([0; diff(gt_vx)]./dt, K, 'same');
gt_ay = conv([0; diff(gt_vy)]./dt, K, 'same');

%--------------------------------------------------------------------------
% Auto-detect stationary window
%--------------------------------------------------------------------------
win   = round(0.5 * 104);
stat_end = win;
for i = 1:win:N-win
    if max(std(double(gyro(i:i+win-1,:)))) > 0.02 || ...
       max(std(double(accel(i:i+win-1,:)))) > 0.05
        break
    end
    stat_end = i + win - 1;
end
stat_end = max(stat_end, 50);

% Moving mask
moving_mask = gt_speed > 0.05;

fprintf('Stationary window: samples 1–%d  (%.2f s)\n', stat_end, t(stat_end));
fprintf('Moving samples:    %d / %d\n\n', sum(moving_mask), N);

%==========================================================================
% PART 1 — STATIONARY BIAS + NOISE
%==========================================================================
accel_bias = mean(double(accel(1:stat_end, :)));
accel_std  = std( double(accel(1:stat_end, :)));
gyro_bias  = mean(double(gyro(1:stat_end, :)));
gyro_std   = std( double(gyro(1:stat_end, :)));

% Gravity axis = max |mean| column
[~, grav_col] = max(abs(accel_bias));

fprintf('==========================================================\n');
fprintf(' PART 1 — STATIONARY BIAS\n');
fprintf('==========================================================\n');
fprintf('\nAccelerometer:\n');
fprintf('  %-8s  %12s  %10s\n','Column','mean(m/s²)','std(m/s²)');
for c = 1:3
    tag = '';
    if c==grav_col; tag = '  ← GRAVITY — skip in EKF'; end
    fprintf('  col %d     %+10.5f   %.6f%s\n', c, accel_bias(c), accel_std(c), tag);
end
fprintf('\nGyroscope:\n');
fprintf('  %-8s  %12s  %10s\n','Column','bias(rad/s)','std(rad/s)');
for c = 1:3
    fprintf('  col %d     %+.6f    %.6f\n', c, gyro_bias(c), gyro_std(c));
end

%==========================================================================
% PART 2 — ACCEL AXIS MAPPING
%==========================================================================
accel_debias = double(accel) - accel_bias;
non_grav = setdiff(1:3, grav_col);

r_ax = zeros(3,1);  r_ay = zeros(3,1);
for c = non_grav
    a_m = accel_debias(moving_mask, c);
    if sum(moving_mask) > 10
        Cx = corrcoef(a_m, gt_ax(moving_mask));  r_ax(c) = Cx(1,2);
        Cy = corrcoef(a_m, gt_ay(moving_mask));  r_ay(c) = Cy(1,2);
    end
end

% Select X axis = highest correlation with ax or ay (whichever is dominant)
[~,ix] = max(abs(r_ax(non_grav)));  accel_x_col = non_grav(ix);
[~,iy] = max(abs(r_ay(non_grav)));  accel_y_col = non_grav(iy);
if accel_x_col == accel_y_col
    other = non_grav(non_grav ~= accel_x_col);
    if ~isempty(other)
        if abs(r_ax(accel_x_col)) >= abs(r_ay(accel_y_col))
            accel_y_col = other(1);
        else
            accel_x_col = other(1);
        end
    end
end

% Determine signs from correlation sign
accel_x_sign = sign(r_ax(accel_x_col));  if accel_x_sign==0; accel_x_sign=1; end
accel_y_sign = sign(r_ay(accel_y_col));  if accel_y_sign==0; accel_y_sign=1; end
sx = sprintf('%+d', accel_x_sign);
sy = sprintf('%+d', accel_y_sign);

fprintf('\n==========================================================\n');
fprintf(' PART 2 — ACCEL AXIS MAPPING\n');
fprintf('==========================================================\n');
fprintf('\n  %-8s  %8s  %8s\n','Column','r(GT_ax)','r(GT_ay)');
for c = non_grav
    mx=''; my='';
    if c==accel_x_col; mx=' ← X_IDX'; end
    if c==accel_y_col; my=' ← Y_IDX'; end
    fprintf('  col %d     %+.3f     %+.3f     %s%s\n', c, r_ax(c), r_ay(c), mx, my);
end
fprintf('\n  Note: if all |r| < 0.2, robot barely accelerated — axis mapping\n');
fprintf('  is unreliable. Cross-check with gravity axis identification.\n');
fprintf('  Non-gravity cols are: %d and %d — assign by process of elimination.\n', ...
    non_grav(1), non_grav(2));

%--------------------------------------------------------------------------
% Noise floor recommendations
%--------------------------------------------------------------------------
accel_noise = max(accel_std(non_grav));
gyro_noise  = min(gyro_std);

fprintf('\n==========================================================\n');
fprintf(' COPY INTO myEKF.m\n');
fprintf('==========================================================\n');
fprintf('  ACCEL_X_IDX  = %d;   ACCEL_X_SIGN = %s;\n', accel_x_col, sx);
fprintf('  ACCEL_Y_IDX  = %d;   ACCEL_Y_SIGN = %s;\n', accel_y_col, sy);
fprintf('\n  %% Gravity axis = col %d  (mean = %.3f m/s²) — never use\n', ...
    grav_col, accel_bias(grav_col));
fprintf('\n  %% Gyro bias (cross-check with calibrate_rotate.m output):\n');
for c=1:3
    fprintf('  %%   col %d: %+.6f rad/s  (std=%.6f)\n', c, gyro_bias(c), gyro_std(c));
end
fprintf('\n  %% Suggested noise values:\n');
fprintf('  R_accel = (%.3f)^2;   %% 3× accel std\n', max(accel_noise*3, 0.05));
fprintf('  R_gyro  = (%.4f)^2;   %% 2× gyro std\n',  max(gyro_noise*2,  0.003));
fprintf('==========================================================\n\n');

%--------------------------------------------------------------------------
% Plots
%--------------------------------------------------------------------------
figure('Name','Straight-Line Calibration','Position',[50 50 1200 750]);
colours = {'r','b','g'};

subplot(2,3,1);
plot(gt_pos(:,1), gt_pos(:,2), 'b-', 'LineWidth',1.5);
hold on;
plot(gt_pos(1,1), gt_pos(1,2), 'go','MarkerSize',10,'MarkerFaceColor','g','DisplayName','Start');
plot(gt_pos(end,1),gt_pos(end,2),'rs','MarkerSize',10,'MarkerFaceColor','r','DisplayName','End');
xlabel('X (m)'); ylabel('Y (m)'); grid on; axis equal;
title('GT trajectory (should be straight line)'); legend;

subplot(2,3,2);
plot(t, gt_speed, 'b','LineWidth',1.2);
hold on;
xline(t(stat_end),'r--',sprintf('stat end (%.1fs)',t(stat_end)),'LineWidth',1.5);
yline(0.05,'g--','motion threshold');
ylabel('Speed (m/s)'); xlabel('Time (s)'); grid on;
title('GT speed — shows stationary vs driving periods');

subplot(2,3,3);
hold on;
for c = non_grav
    plot(t, accel_debias(:,c), colours{c}, 'LineWidth',0.8, ...
         'DisplayName', sprintf('col%d  r_ax=%.2f  r_ay=%.2f', c, r_ax(c), r_ay(c)));
end
ylabel('Accel debiased (m/s²)'); xlabel('Time (s)'); grid on;
legend('Location','best');
title(sprintf('Accel (non-gravity)  →  X=col%d%s  Y=col%d%s', ...
    accel_x_col, sx, accel_y_col, sy));

subplot(2,3,4);
hold on;
for c = 1:3
    plot(t(1:stat_end), double(accel(1:stat_end,c)), colours{c}, ...
         'DisplayName', sprintf('col%d  bias=%.4f', c, accel_bias(c)));
end
legend; grid on;
ylabel('Accel (m/s²)'); xlabel('Time (s)');
title(sprintf('Stationary accel (%d samples)', stat_end));

subplot(2,3,5);
hold on;
for c = 1:3
    plot(t(1:stat_end), rad2deg(double(gyro(1:stat_end,c))), colours{c}, ...
         'DisplayName', sprintf('col%d  bias=%.5f', c, gyro_bias(c)));
end
legend; grid on;
ylabel('Gyro (deg/s)'); xlabel('Time (s)');
title('Stationary gyro bias');

subplot(2,3,6);
yyaxis left;
plot(t, accel_debias(:,accel_x_col)*accel_x_sign, 'r', ...
     'DisplayName', sprintf('Accel col%d×sign', accel_x_col));
ylabel('Accel (m/s²)');
yyaxis right;
dom_gt = gt_ay;  % straight line motion is typically in Y
if max(abs(gt_ax)) > max(abs(gt_ay)); dom_gt = gt_ax; end
plot(t, dom_gt, 'b--', 'DisplayName', 'GT accel (dominant axis)');
ylabel('GT accel (m/s²)'); xlabel('Time (s)'); grid on;
legend('Location','best');
title('Accel col vs GT acceleration — should correlate during motion');

sgtitle('Straight-Line Calibration');
