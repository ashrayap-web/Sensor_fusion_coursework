clear; close all; clc;
clear myEKF   % REQUIRED every run — resets persistent variables

DATA_FILE = 'task2_4.mat';   % <-- change to your task file
data = load(DATA_FILE);
out  = data.out;

%--------------------------------------------------------------------------
% Extract streams
%--------------------------------------------------------------------------
acc_all    = squeeze(out.Sensor_ACCEL.signals.values)';
gyro_all   = squeeze(out.Sensor_GYRO.signals.values)';
mag_all    = squeeze(out.Sensor_MAG.signals.values)';
tof1_all   = out.Sensor_ToF1.signals.values;
tof2_all   = out.Sensor_ToF2.signals.values;
tof3_all   = out.Sensor_ToF3.signals.values;
temp_all   = out.Sensor_Temp.signals.values;
lp_acc_all = out.Sensor_LP_ACCEL.signals.values;

gt_pos  = out.GT_position.signals.values;
gt_quat = out.GT_rotation.signals.values;
N_gt    = size(gt_quat,1);
N_imu   = size(acc_all,1);

% All signals (IMU + GT) are logged at the same 200 Hz rate in Simulink.
tout      = out.tout;
t_sim_end = tout(end);
N         = N_imu;   % N_imu == N_gt (same timebase)

fprintf('tout end: %.2f s  |  Samples: %d (IMU=%d, GT=%d)  |  Rate: 200 Hz\n', ...
        t_sim_end, N, N_imu, N_gt);

gt_yaw = zeros(N_gt,1);
for i = 1:N_gt
    W_=gt_quat(i,1); QX_=gt_quat(i,2); QY_=gt_quat(i,3); QZ_=gt_quat(i,4);
    gt_yaw(i) = atan2(2*(W_*QZ_+QX_*QY_), 1-2*(QY_^2+QZ_^2));
end

t_vec = (0:N-1)/200;   % all data at 200 Hz
theta0   = gt_yaw(1);
gt_init  = [gt_pos(1,1), gt_pos(1,2), theta0];

fprintf('GT start: x=%.3f m, y=%.3f m, quat_theta=%.1f deg\n', ...
        gt_init(1), gt_init(2), rad2deg(theta0));

% =========================================================================
% SANITY CHECK: first ToF readings vs expected wall distances
% The EKF will brute-force the correct heading, but this print lets you
% verify the sensor mounting is still correct.
% =========================================================================
arena.x_min=-1.22; arena.x_max=1.22; arena.y_min=-1.22; arena.y_max=1.22;
fprintf('\nFirst valid ToF readings:\n');
tof_all_c = {tof1_all, tof2_all, tof3_all};
for s=1:3
    for k=1:size(tof_all_c{s},1)
        r=tof_all_c{s}(k,:);
        if ~any(isnan(r))&&r(4)==0&&r(1)>=0.05&&r(1)<=2.5
            fprintf('  ToF%d first valid = %.3f m\n', s, r(1));
            break;
        end
    end
end

% Expected wall distances from GT start in all 4 directions
fprintf('Expected walls from GT start:\n');
dir_names={'Forward(0)','Left(+90)','Right(-90)','Backward(180)'};
offsets=[0, pi/2, -pi/2, pi];
for d=1:4
    ra=theta0+offsets(d);
    rdx=cos(ra); rdy=sin(ra); dist=Inf;
    dists=[];
    if abs(rdx)>1e-9
        t=(arena.x_max-gt_pos(1,1))/rdx;
        if t>0&&gt_pos(1,2)+t*rdy>=arena.y_min&&gt_pos(1,2)+t*rdy<=arena.y_max; dists(end+1)=t; end
        t=(arena.x_min-gt_pos(1,1))/rdx;
        if t>0&&gt_pos(1,2)+t*rdy>=arena.y_min&&gt_pos(1,2)+t*rdy<=arena.y_max; dists(end+1)=t; end
    end
    if abs(rdy)>1e-9
        t=(arena.y_max-gt_pos(1,2))/rdy;
        if t>0&&gt_pos(1,1)+t*rdx>=arena.x_min&&gt_pos(1,1)+t*rdx<=arena.x_max; dists(end+1)=t; end
        t=(arena.y_min-gt_pos(1,2))/rdy;
        if t>0&&gt_pos(1,1)+t*rdx>=arena.x_min&&gt_pos(1,1)+t*rdx<=arena.x_max; dists(end+1)=t; end
    end
    if ~isempty(dists); dist=min(dists); end
    fprintf('  %-16s = %.3f m  (using quat theta=%.1f deg)\n', dir_names{d}, dist, rad2deg(theta0));
end
fprintf('(Note: if ToF readings match a different set of directions, the EKF\n');
fprintf(' brute-force heading will still find the correct initial theta.)\n\n');

% =========================================================================
% RUN EKF
% =========================================================================
N_loop = N;
X_History = zeros(8,N_loop);

for k=1:N_loop
    mag=get_dat(mag_all,k); t1=get_dat(tof1_all,k);
    t2=get_dat(tof2_all,k); t3=get_dat(tof3_all,k);
    tmp=get_dat(temp_all,k); lp=get_dat(lp_acc_all,k);
    if k==1
        % gt_init is optional — only theta is used as a fallback heading hint.
        % Position is determined automatically from wall midpoint + ToF scan.
        % Pass gt_init for diagnostic comparison, or omit entirely:
        %   [X_Est,~]=myEKF(acc_all(k,:),gyro_all(k,:),mag,t1,t2,t3,tmp,lp);
        [X_Est,~]=myEKF(acc_all(k,:),gyro_all(k,:),mag,t1,t2,t3,tmp,lp,gt_init);
    else
        [X_Est,~]=myEKF(acc_all(k,:),gyro_all(k,:),mag,t1,t2,t3,tmp,lp);
    end
    X_History(:,k)=X_Est;
end

% Direct comparison — IMU and GT are on the same 200 Hz timebase
gt_xy  = gt_pos(1:N, 1:2);
ekf_xy = X_History([1,4], 1:N)';
rmse   = sqrt(mean(sum((ekf_xy - gt_xy).^2, 2)));
err_x  = ekf_xy(:,1) - gt_xy(:,1);
err_y  = ekf_xy(:,2) - gt_xy(:,2);

fprintf('=== RESULTS ===\n');
fprintf('RMSE:    %.4f m\n', rmse);
fprintf('X error: mean=%+.3f m,  std=%.3f m\n', mean(err_x), std(err_x));
fprintf('Y error: mean=%+.3f m,  std=%.3f m\n', mean(err_y), std(err_y));
fprintf('EKF init theta: %.1f deg  (quat gave %.1f deg)\n', ...
        rad2deg(X_History(7,1)), rad2deg(theta0));

% =========================================================================
% PLOTS
% =========================================================================
figure('Name','XY Trajectory');
plot(gt_pos(:,1),gt_pos(:,2),'b--','LineWidth',1.5,'DisplayName','Ground Truth'); hold on;
plot(X_History(1,:),X_History(4,:),'r','LineWidth',1.2,'DisplayName','EKF Estimate');
plot(gt_pos(1,1),gt_pos(1,2),'go','MarkerSize',10,'LineWidth',2,'DisplayName','Start');
axis equal; grid on;
title(sprintf('XY Trajectory — RMSE: %.4f m',rmse));
xlabel('X [m]'); ylabel('Y [m]'); legend('Location','best');

figure('Name','X and Y Error over Time');
subplot(2,1,1);
plot(t_vec,err_x,'r','LineWidth',1.0); hold on; yline(0,'k--'); grid on;
ylabel('X error [m]'); title(sprintf('X error  mean=%+.3f m',mean(err_x)));
subplot(2,1,2);
plot(t_vec,err_y,'b','LineWidth',1.0); hold on; yline(0,'k--'); grid on;
ylabel('Y error [m]'); xlabel('Time [s]');
title(sprintf('Y error  mean=%+.3f m',mean(err_y)));

figure('Name','Heading over Time');
plot(t_vec,rad2deg(gt_yaw),'b--','LineWidth',1.5,'DisplayName','GT Yaw'); hold on;
plot(t_vec,rad2deg(X_History(7,:)),'r','LineWidth',1.2,'DisplayName','EKF Theta');
grid on; xlabel('Time [s]'); ylabel('Heading [deg]');
title('Heading over Time'); legend('Location','best');

function v = get_dat(arr,k)
    if k<=size(arr,1); v=arr(k,:); else; v=NaN(1,size(arr,2)); end
end