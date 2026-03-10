%==========================================================================
% test_myEKF.m  —  Run EKF, print RMSE, show 2D trajectory plot
%==========================================================================
clear; close all; clc;

%--------------------------------------------------------------------------
% CONFIGURATION
%--------------------------------------------------------------------------
DATA_FILE = 'task2_1 1.mat';   % <-- set your .mat path here
TASK_NAME = 'Task 1';               % <-- 'Task 1' or 'Task 2'

%--------------------------------------------------------------------------
% LOAD DATA
%--------------------------------------------------------------------------
fprintf('Loading: %s\n', DATA_FILE);
if ~isfile(DATA_FILE)
    error('File not found: %s', DATA_FILE);
end
data   = load(DATA_FILE);
fields = fieldnames(data);
if isfield(data, 'out')
    out = data.out;
elseif length(fields) == 1
    out = data.(fields{1});
else
    fprintf('Fields in .mat:\n');
    for i = 1:length(fields); fprintf('  %d: %s\n', i, fields{i}); end
    choice = input('Enter field number: ');
    out = data.(fields{choice});
end

%--------------------------------------------------------------------------
% RUN EKF
%--------------------------------------------------------------------------
fprintf('Running myEKF...\n');
tic;
[X_Est, P_Est, GT] = myEKF2(out);
fprintf('Done in %.2f s\n\n', toc);

%--------------------------------------------------------------------------
% UNPACK
%--------------------------------------------------------------------------
x_est     = X_Est(1,:);
y_est     = X_Est(4,:);
theta_est = X_Est(7,:);
x_gt      = GT(1,:);
y_gt      = GT(2,:);
theta_gt  = GT(3,:);
N         = length(x_est);

% Time axis
try
    t = squeeze(out.Sensor_Time.signals.values);
    t = t(:)' - t(1);
catch
    t = (0:N-1) / 104;
end

%--------------------------------------------------------------------------
% RMSE
%--------------------------------------------------------------------------
pos_err   = sqrt((x_est - x_gt).^2 + (y_est - y_gt).^2);
rmse_pos  = sqrt(mean(pos_err.^2));
rmse_x    = sqrt(mean((x_est - x_gt).^2));
rmse_y    = sqrt(mean((y_est - y_gt).^2));
theta_err = arrayfun(@(a) wrap_angle_local(a), theta_est - theta_gt);
rmse_th   = sqrt(mean(theta_err.^2));

fprintf('============================================================\n');
fprintf(' RESULTS — %s\n', TASK_NAME);
fprintf('============================================================\n');
fprintf(' RMSE Position (Euclidean) : %.4f m\n',    rmse_pos);
fprintf(' RMSE X                    : %.4f m\n',    rmse_x);
fprintf(' RMSE Y                    : %.4f m\n',    rmse_y);
fprintf(' RMSE Heading              : %.4f rad  (%.2f deg)\n', rmse_th, rad2deg(rmse_th));
fprintf('------------------------------------------------------------\n');
fprintf(' Max Position Error        : %.4f m\n',    max(pos_err));
fprintf(' Final Position Error      : %.4f m\n',    pos_err(end));
fprintf(' Run duration              : %.1f s  (%d samples)\n', t(end), N);
fprintf('============================================================\n\n');

%--------------------------------------------------------------------------
% 2D TRAJECTORY PLOT
%--------------------------------------------------------------------------
figure('Name', sprintf('%s — 2D Trajectory', TASK_NAME), ...
       'Position', [100, 100, 750, 650]);

% Arena background
arena_x = [-1.2,  1.2,  1.2, -1.2, -1.2];
arena_y = [-1.2,-1.2, 1.2, 1.2,-1.2];
patch(arena_x, arena_y, [0.96 0.96 0.96], ...
      'EdgeColor', [0.2 0.2 0.2], 'LineWidth', 2.5, 'FaceAlpha', 0.4);
hold on; grid on; axis equal;

% Trajectories
h_gt  = plot(x_gt,  y_gt,  'b-', 'LineWidth', 2.2, 'DisplayName', 'Ground Truth');
h_est = plot(x_est, y_est, 'r-', 'LineWidth', 1.6, 'DisplayName', 'EKF Estimate');

% Start marker (shared — both should start same place)
plot(x_gt(1), y_gt(1), 'go', 'MarkerSize', 14, 'MarkerFaceColor', [0.1 0.8 0.1], ...
     'DisplayName', 'Start');

% End markers
plot(x_gt(end),  y_gt(end),  'bs', 'MarkerSize', 11, 'MarkerFaceColor', 'b', ...
     'DisplayName', 'GT End');
plot(x_est(end), y_est(end), 'r^', 'MarkerSize', 11, 'MarkerFaceColor', 'r', ...
     'DisplayName', 'EKF End');

% Heading arrows every ~1.5 s
arrow_step = max(1, round(1.5 / (t(end)/N)));
arrow_len  = 0.07;
for i = 1:arrow_step:N
    quiver(x_est(i), y_est(i), ...
           arrow_len*cos(theta_est(i)), ...
           arrow_len*sin(theta_est(i)), ...
           0, 'Color', [0.75 0.1 0.1], 'MaxHeadSize', 2.5, 'LineWidth', 0.8);
end

% Labels
xlabel('X position (m)', 'FontSize', 13);
ylabel('Y position (m)', 'FontSize', 13);
title(sprintf('%s  —  2D Trajectory\nRMSE_{pos} = %.4f m     RMSE_{\\theta} = %.4f rad (%.1f°)', ...
      TASK_NAME, rmse_pos, rmse_th, rad2deg(rmse_th)), 'FontSize', 13);
legend([h_gt, h_est], 'Location', 'best', 'FontSize', 11);

xlim([-1.2, 1.2]);
ylim([-1.2, 1.2]);
text(-0.95, -0.68, 'Arena', 'FontSize', 9, 'Color', [0.5 0.5 0.5]);

%--------------------------------------------------------------------------
% SAVE .mat for submission
%--------------------------------------------------------------------------
task_tag = lower(strrep(TASK_NAME, ' ', '_'));
save_name = sprintf('%s_estimated.mat', task_tag);
save(save_name, 'X_Est', 'P_Est');
fprintf('Saved: %s\n', save_name);

%--------------------------------------------------------------------------
% LOCAL HELPER
%--------------------------------------------------------------------------
function a = wrap_angle_local(a)
    a = mod(a + pi, 2*pi) - pi;
end