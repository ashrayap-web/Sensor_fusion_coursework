% show_calibration.m — Display all EKF calibration parameters and live sensor stats
% Run this to see a summary of all values used by myEKF.
clear; clc;

fprintf('============================================================\n');
fprintf('  EKF CALIBRATION SUMMARY\n');
fprintf('============================================================\n\n');

%% Read current values from myEKF.m source
txt = fileread('myEKF.m');

fprintf('--- GYROSCOPE ---\n');
fprintf('  Yaw axis column:    %s\n', xval(txt, 'GYRO_Z_IDX'));
fprintf('  Yaw axis sign:      %s\n', xval(txt, 'GYRO_Z_SIGN'));
fprintf('  Static bias:        %s rad/s\n', xval(txt, 'gyro_bias_z'));
fprintf('  R_gyro:             %s rad^2\n', xval(txt, 'R_gyro'));
fprintf('  Still threshold:    %s rad/s\n', xval(txt, 'OMEGA_STILL_THRESH'));

fprintf('\n--- ACCELEROMETER ---\n');
fprintf('  Forward axis col:   %s  (sign: %s)\n', xval(txt,'ACCEL_X_IDX'), xval(txt,'ACCEL_X_SIGN'));
fprintf('  Forward bias:       %s m/s^2\n', xval(txt, 'ACCEL_BIAS_X'));
fprintf('  Lateral axis col:   %s  (sign: %s)\n', xval(txt,'ACCEL_Y_IDX'), xval(txt,'ACCEL_Y_SIGN'));
fprintf('  Lateral bias:       %s m/s^2\n', xval(txt, 'ACCEL_BIAS_Y'));
fprintf('  R_accel:            %s (m/s^2)^2\n', xval(txt, 'R_accel'));

fprintf('\n--- ToF SENSORS ---\n');
fprintf('  ToF1: Left       angle_offset =  pi/2\n');
fprintf('  ToF2: Backward   angle_offset =  pi\n');
fprintf('  ToF3: Right      angle_offset = -pi/2\n');
fprintf('  R_tof:             %s m^2\n', xval(txt, 'R_tof'));

fprintf('\n--- MAGNETOMETER ---\n');
fprintf('  Functional:         %s\n', xval(txt, 'MAG_FUNCTIONAL'));
fprintf('  Hard-iron offset:   %s T\n', xval(txt, 'mag_hard_iron'));
fprintf('  Soft-iron scale:    %s T\n', xval(txt, 'mag_soft_iron'));
fprintf('  Yaw offset:         %s rad\n', xval(txt, 'mag_yaw_offset'));
fprintf('  R_mag:              %s rad^2\n', xval(txt, 'R_mag'));

fprintf('\n--- PROCESS NOISE Q ---\n');
names = {'x pos','x vel','x acc','y pos','y vel','y acc','heading','omega'};
for i=1:8
    tok=regexp(txt, sprintf('Q\\(%d,%d\\)\\s*=\\s*([\\d\\.e\\-\\+]+)',i,i), 'tokens','once');
    if ~isempty(tok); fprintf('  Q(%d,%d) = %-10s  (%s)\n',i,i,tok{1},names{i}); end
end

fprintf('\n--- MOTION MODEL ---\n');
fprintf('  Sample rate:        200 Hz (dt = 1/200)\n');
fprintf('  Vel damping alpha:  %s\n', xval(txt, 'alpha_v'));

fprintf('\n--- INITIALISATION ---\n');
tok=regexp(txt,'USE_GT_INIT\s*=\s*(true|false)\s*;','tokens','once');
if ~isempty(tok); fprintf('  USE_GT_INIT:        %s\n',tok{1}); else; fprintf('  USE_GT_INIT:        (not found)\n'); end
fprintf('  Heading scan:       2-pass (1 deg coarse -> 0.05 deg fine)\n');

fprintf('\n--- TUNING ---\n');
tok=regexp(txt,'gate\s*=\s*min\(3\*sqrt\(S_tof\),\s*([^)]+)\)','tokens','once');
if ~isempty(tok); fprintf('  Innovation gate:    %s m\n', strtrim(tok{1})); end
fprintf('  ZUPT_R:             %s (m/s)^2\n', xval(txt, 'ZUPT_R'));
fprintf('  ZAUT_R:             %s (m/s^2)^2\n', xval(txt, 'ZAUT_R'));

fprintf('\n--- ARENA ---\n');
fprintf('  X range: [-1.22, +1.22] m\n');
fprintf('  Y range: [-1.22, +1.22] m\n');

%% ================================================================
%  LIVE SENSOR STATS FROM CALIBRATION FILES
%% ================================================================
fprintf('\n============================================================\n');
fprintf('  LIVE SENSOR STATISTICS\n');
fprintf('============================================================\n');

if exist('calib1_rotate.mat','file')
    d=load('calib1_rotate.mat'); o=d.out;
    gyro=squeeze(o.Sensor_GYRO.signals.values)'; N=size(gyro,1);
    stat_end=min(400,N); gs=gyro(1:stat_end,:);
    fprintf('\n--- calib1_rotate (gyro/mag calibration) ---\n');
    fprintf('  Duration: %.1fs  Samples: %d\n', N/200, N);
    fprintf('  Gyro static bias (first 2s):\n');
    for c=1:3; fprintf('    Col %d: mean=%+.6f  std=%.6f rad/s\n',c,mean(gs(:,c)),std(gs(:,c))); end
    fprintf('  Gyro full-run range:\n');
    for c=1:3; fprintf('    Col %d: [%+.4f, %+.4f] rad/s\n',c,min(gyro(:,c)),max(gyro(:,c))); end

    % GT yaw for reference
    gt_quat=o.GT_rotation.signals.values; gt_yaw=zeros(N,1);
    for i=1:N; W_=gt_quat(i,1);X_=gt_quat(i,2);Y_=gt_quat(i,3);Z_=gt_quat(i,4);
        gt_yaw(i)=atan2(2*(W_*Z_+X_*Y_),1-2*(Y_^2+Z_^2)); end
    gt_uw=unwrap(gt_yaw);
    fprintf('  GT total rotation: %.1f deg (%.1f turns)\n', rad2deg(gt_uw(end)-gt_uw(1)),(gt_uw(end)-gt_uw(1))/(2*pi));

    % Mag
    mag=squeeze(o.Sensor_MAG.signals.values)'; mu=mag([true;any(diff(mag)~=0,2)],:);
    fprintf('  Mag unique readings: %d\n', size(mu,1));
    for c=1:min(3,size(mu,2)); fprintf('    Col %d: range=[%.2e, %.2e]  spread=%.2e T\n',c,min(mu(:,c)),max(mu(:,c)),max(mu(:,c))-min(mu(:,c))); end
    Mx=(mu(:,1)-mean(mu(:,1)))/(range(mu(:,1))/2);
    My=(mu(:,2)-mean(mu(:,2)))/(range(mu(:,2))/2);
    fprintf('  Mag heading std: %.1f deg  (>30 = unusable)\n', rad2deg(std(atan2(My,Mx))));
end

if exist('calib2_straight.mat','file')
    d2=load('calib2_straight.mat'); o2=d2.out;
    acc=squeeze(o2.Sensor_ACCEL.signals.values)'; N2=size(acc,1);
    stat_end2=min(400,N2); as=acc(1:stat_end2,:);
    fprintf('\n--- calib2_straight (accel/ToF calibration) ---\n');
    fprintf('  Duration: %.1fs  Samples: %d\n', N2/200, N2);
    fprintf('  Accel static bias (first 2s):\n');
    for c=1:3; fprintf('    Col %d: mean=%+.4f  std=%.4f m/s^2\n',c,mean(as(:,c)),std(as(:,c))); end

    % ToF stats
    tof_c={o2.Sensor_ToF1.signals.values, o2.Sensor_ToF2.signals.values, o2.Sensor_ToF3.signals.values};
    for s=1:3
        td=tof_c{s}; valid=td(:,4)==0 & td(:,1)>=0.05 & td(:,1)<=2.5;
        vr=td(valid,1); ch=[true;diff(vr)~=0];
        fprintf('  ToF%d: valid=%d unique=%d range=[%.3f,%.3f]m signal=[%.0f,%.0f]\n', ...
            s,sum(valid),sum(ch),min(vr),max(vr),min(td(valid,3)),max(td(valid,3)));
    end
end

%% ================================================================
%  DATASET OVERVIEW
%% ================================================================
fprintf('\n============================================================\n');
fprintf('  DATASET OVERVIEW\n');
fprintf('============================================================\n');
fprintf('  %-18s  %6s  %8s  %8s  %10s  %8s\n','Dataset','N','GT_x0','GT_y0','yaw0(deg)','dur(s)');
datasets={'task1_1 1.mat','task1_2 1.mat','task1_3.mat','task2_1 1.mat','task2_2 1.mat','task2_3 1.mat','task2_4.mat'};
for di=1:numel(datasets)
    if ~exist(datasets{di},'file'); continue; end
    dd=load(datasets{di}); oo=dd.out;
    Nd=size(squeeze(oo.Sensor_ACCEL.signals.values)',1);
    gtp=oo.GT_position.signals.values; gtq=oo.GT_rotation.signals.values;
    W_=gtq(1,1);X_=gtq(1,2);Y_=gtq(1,3);Z_=gtq(1,4);
    yaw0=atan2(2*(W_*Z_+X_*Y_),1-2*(Y_^2+Z_^2));
    fprintf('  %-18s  %6d  %8.3f  %8.3f  %10.1f  %8.1f\n',datasets{di},Nd,gtp(1,1),gtp(1,2),rad2deg(yaw0),Nd/200);
end

fprintf('\n============================================================\n');

function val=xval(txt,name)
    tok=regexp(txt,[name '\s*=\s*([^;]+);'],'tokens','once');
    if ~isempty(tok); val=strtrim(tok{1}); else; val='(not found)'; end
end
