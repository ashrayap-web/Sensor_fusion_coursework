function [X_Est, P_Est, GT] = myEKF(out)
% myEKF - Extended Kalman Filter for sub-terranean robot navigation
% COMP0217 Final Project - University College London
%
% State vector (all in WORLD frame):
%   x_state = [x, vx, ax, y, vy, ay, theta, omega]
%              1   2   3  4   5   6     7      8

%==========================================================================
% ARENA CONFIGURATION
%==========================================================================
% Arena bounds — measured from actual GT start position
% Robot starts at y=-0.933, confirming y_min is approximately -1.0
% Adjust x/y max based on your actual arena dimensions from the brief
arena.x_min = -1.2;
arena.x_max =  1.2;
arena.y_min = -1.2;
arena.y_max =  1.2;

%==========================================================================
% IMU AXIS REMAPPING
%--------------------------------------------------------------------------
% Run diagnose_axes.m  → copy suggested values here.
% Run calibrate_gyro.m → copy GYRO_SCALE here.
%
% Quick manual check:
%   theta0 printed at startup should match the robot's real heading.
%   If EKF curves RIGHT when GT goes STRAIGHT UP → flip GYRO_Z_SIGN.
%   If EKF goes RIGHT when GT goes UP → swap ACCEL_X/Y indices.
%--------------------------------------------------------------------------
% Axis mapping confirmed from quick_debug.m output:
%   col 1 = gravity axis (mean=10 m/s²) — NOT used
%   col 2 = body-X forward (r(vx)=+0.153, highest |r(vx)|)
%   col 3 = body-Y left    (r(vy) weak but col2 is taken; col3 is lateral)
%   Gyro col 3 = yaw rate  (bias=-0.00119 rad/s, very small — good)
%   Quaternion: pitch-axis convention (conv 3) selected automatically
ACCEL_X_IDX  = 2;   ACCEL_X_SIGN =  1;   % body-forward (col1=gravity, skip)
ACCEL_Y_IDX  = 3;   ACCEL_Y_SIGN =  1;   % body-left lateral
GYRO_Z_IDX   = 3;   GYRO_Z_SIGN  =  -1;   % yaw rate — bias only -0.00119 rad/s
GYRO_SCALE   = 1.0; % update from calibrate_gyro.m if scale != 1.0

%==========================================================================
% SENSOR MOUNTING — body frame offsets (metres)
% ToF1=forward, ToF2=left, ToF3=right
%==========================================================================
sensors(1).offset_x =  0.10;  sensors(1).offset_y =  0.00;  sensors(1).angle_offset =  0.00;
sensors(2).offset_x =  0.00;  sensors(2).offset_y =  0.10;  sensors(2).angle_offset =  pi/2;
sensors(3).offset_x =  0.00;  sensors(3).offset_y = -0.10;  sensors(3).angle_offset = -pi/2;

%==========================================================================
% NOISE PARAMETERS
%==========================================================================
% Process noise
Q = zeros(8,8);
Q(1,1) = 5; % x uncertainty
Q(2,2) = 1.0; %0.01    % vx diffusion
Q(3,3) = 8.0;     % ax uncertainty (large — don't trust accel for position)
Q(4,4) = 0.1; % y uncertainty
Q(5,5) = 0.01;    % vy diffusion
Q(6,6) = 5.0;     % ay uncertainty
Q(7,7) = 0.0; % theta
Q(8,8) = 0.25;    % omega uncertainty

% Measurement noise
R_accel = (1.33)^2;    % accel measurement — large, only corrects ax/ay
R_gyro  = (0.3)^2;  % gyro very trusted
R_tof   = (0.02)^2;   % ToF accurate when valid

USE_MAG = false;

%==========================================================================
% EXTRACT RAW SENSOR DATA
%==========================================================================
accel = squeeze(out.Sensor_ACCEL.signals.values)';  % [N x 3]
gyro  = squeeze(out.Sensor_GYRO.signals.values)';   % [N x 3]
mag   = squeeze(out.Sensor_MAG.signals.values)';    % [N x 3]

tof1 = out.Sensor_ToF1.signals.values;   % [N x 4]
tof2 = out.Sensor_ToF2.signals.values;
tof3 = out.Sensor_ToF3.signals.values;

sens_time = squeeze(out.Sensor_Time.signals.values);
sens_time = sens_time(:);
N = length(sens_time);

%==========================================================================
% EXTRACT GROUND TRUTH
%==========================================================================
gt_pos = out.GT_position.signals.values;   % [M x 3]
gt_rot = out.GT_rotation.signals.values;   % [M x 4] w,x,y,z
gt_time = squeeze(out.GT_time.signals.values);
gt_time = gt_time(:);

% Quaternion → yaw
% PhaseSpace outputs quaternion as [w, x, y, z]
% Standard yaw = atan2(2(wz+xy), 1-2(y²+z²))
% BUT PhaseSpace may use a different world frame (Z-up vs Y-up).
% We try the convention that gives the smallest heading range for Task 1
% (straight line drive should have <15 deg heading change).
M_gt = size(gt_rot, 1);
gt_theta = zeros(M_gt, 1);

% Convention A: standard Z-up  yaw = atan2(2(wz+xy), 1-2(y²+z²))
% Convention B: Y-up world     yaw = atan2(2(wx+yz), 1-2(x²+z²)) -- roll
% Convention C: pitch angle    yaw = atan2(2(wy+xz), 1-2(x²+y²))
% Convention D: negate Z       yaw = atan2(2(-wz-xy), 1-2(y²+z²))
candidates = zeros(M_gt, 4);
for i = 1:M_gt
    w=gt_rot(i,1); qx=gt_rot(i,2); qy=gt_rot(i,3); qz=gt_rot(i,4);
    candidates(i,1) = atan2( 2*(w*qz + qx*qy),  1 - 2*(qy^2 + qz^2)); % standard
    candidates(i,2) = atan2( 2*(w*qx + qy*qz),  1 - 2*(qx^2 + qz^2)); % roll-as-yaw
    candidates(i,3) = atan2( 2*(w*qy + qx*qz),  1 - 2*(qx^2 + qy^2)); % pitch-as-yaw
    candidates(i,4) = atan2(-2*(w*qz + qx*qy),  1 - 2*(qy^2 + qz^2)); % negated Z
end

% Convention 3 (pitch-axis) confirmed correct from quick_debug.m:
%   range=1.2 deg for straight-line Task 1, start=-0.4 deg (facing +X) ✓
%   Formula: yaw = atan2(2(wy+xz), 1-2(x²+y²))
conv_names = {'standard-Z','roll-axis','pitch-axis','neg-Z'};
QUAT_CONVENTION = 3;  % locked in — change only if data suggests otherwise
fprintf('Quaternion convention: %s (conv %d)\n', conv_names{QUAT_CONVENTION}, QUAT_CONVENTION);
gt_theta = candidates(:, QUAT_CONVENTION);

% GT is already on the same time grid as sensor data (same N).
% If sizes differ, interpolate; otherwise use directly.
if size(gt_pos,1) == N
    % Already aligned — use directly
    GT = zeros(3, N);
    GT(1,:) = gt_pos(:,1)';
    GT(2,:) = gt_pos(:,2)';
    GT(3,:) = gt_theta';
else
    % Different lengths — interpolate GT onto sensor time grid
    [gt_time_u, ia] = unique(gt_time, 'stable');
    gt_pos_u   = gt_pos(ia,:);
    gt_theta_u = gt_theta(ia);
    % Remove any non-finite timestamps
    valid = isfinite(gt_time_u);
    gt_time_u  = gt_time_u(valid);
    gt_pos_u   = gt_pos_u(valid,:);
    gt_theta_u = gt_theta_u(valid);
    sc = max(gt_time_u(1), min(gt_time_u(end), sens_time));
    GT = zeros(3, N);
    GT(1,:) = interp1(gt_time_u, gt_pos_u(:,1), sc, 'linear','extrap')';
    GT(2,:) = interp1(gt_time_u, gt_pos_u(:,2), sc, 'linear','extrap')';
    GT(3,:) = interp1(gt_time_u, gt_theta_u,    sc, 'linear','extrap')';
end

%==========================================================================
% GYRO BIAS — estimated from stationary period at start of data
% The first ~0.5s of any run the robot is stationary.
% Use MORE samples (200 @ 104Hz ≈ 2s) for a better bias estimate.
%==========================================================================
n_bias = min(200, N);
raw_gz = GYRO_Z_SIGN * double(gyro(1:n_bias, GYRO_Z_IDX));
gyro_bias_z = -0.031618;
gyro_bias_std = 0.109365;
fprintf('Gyro bias: %.6f rad/s  (std=%.6f)\n', gyro_bias_z, gyro_bias_std);

% Update gyro noise based on measured bias stability
R_gyro = max((0.003)^2, gyro_bias_std^2 * 2);

%==========================================================================
% INITIALISE STATE
%==========================================================================
% Start position = first GT sample directly.
% Do NOT average — averaging biases the start toward early motion.
% GT(1) is the ground truth position at t=0, which is exactly where
% the robot is when the EKF begins, so x_state(1) = GT(1).
x0     = GT(1, 1);
y0     = GT(2, 1);
theta0 = GT(3, 1);

%==========================================================================
% STARTUP DIAGNOSTICS — printed once to help with debugging
%==========================================================================
fprintf('Init: x=%.3f  y=%.3f  theta=%.3f rad (%.1f deg)\n', ...
        x0, y0, theta0, rad2deg(theta0));
fprintf('  → If EKF curves wrong way, flip GYRO_Z_SIGN\n');
tof1_start = double(out.Sensor_ToF1.signals.values(1,1));
tof2_start = double(out.Sensor_ToF2.signals.values(1,1));
tof3_start = double(out.Sensor_ToF3.signals.values(1,1));
fprintf('ToF at t=0: ToF1=%.3fm  ToF2=%.3fm  ToF3=%.3fm\n', ...
        tof1_start, tof2_start, tof3_start);

x_state = [x0; 0; 0; y0; 0; 0; theta0; 0];

% Tight on position (from GT), loose on velocity/accel
P = diag([0.005, 1.0, 4.0, ...
          0.005, 1.0, 4.0, ...
          0.005, 1.0]);

%==========================================================================
% OUTPUT STORAGE
%==========================================================================
X_Est = zeros(8, N);
P_Est = zeros(8, 8, N);

% Write initial state at k=1 BEFORE the loop so the EKF path
% starts exactly at the init position — not one step ahead
X_Est(:,1)   = x_state;
P_Est(:,:,1) = P;

%==========================================================================
% MAIN EKF LOOP
%==========================================================================
for k = 2:N   % start at 2 — k=1 already written above as init state

    % --- dt ---
    dt = double(sens_time(k)) - double(sens_time(k-1));
    if dt <= 0 || dt > 0.5;  dt = 1/104;  end

    % --- PREDICTION ---
    F = [1, dt, 0.5*dt^2, 0,  0,        0, 0,   0;
         0,  1,       dt, 0,  0,        0, 0,   0;
         0,  0,        1, 0,  0,        0, 0,   0;
         0,  0,        0, 1, dt, 0.5*dt^2, 0,   0;
         0,  0,        0, 0,  1,       dt, 0,   0;
         0,  0,        0, 0,  0,        1, 0,   0;
         0,  0,        0, 0,  0,        0, 1,  dt;
         0,  0,        0, 0,  0,        0, 0,   1];

    x_pred = F * x_state;
    P_pred = F * P * F' + Q;

    % --- GYROSCOPE UPDATE ---
    % Gyro directly measures omega — this is the most reliable sensor
    omega_raw  = GYRO_Z_SIGN * double(gyro(k, GYRO_Z_IDX));
    omega_meas = (omega_raw - gyro_bias_z) * GYRO_SCALE;
    H_gyro = [0, 0, 0, 0, 0, 0, 0, 1];
    [x_pred, P_pred] = ekf_update(x_pred, P_pred, omega_meas, x_pred(8), H_gyro, R_gyro);

    % --- ACCELEROMETER UPDATE (ax/ay only — does NOT affect x,y position) ---
    ax_body = ACCEL_X_SIGN * double(accel(k, ACCEL_X_IDX));
    ay_body = ACCEL_Y_SIGN * double(accel(k, ACCEL_Y_IDX));
    c = cos(x_pred(7));  s = sin(x_pred(7));
    ax_world =  c*ax_body - s*ay_body;
    ay_world =  s*ax_body + c*ay_body;
    [x_pred, P_pred] = ekf_update(x_pred, P_pred, ax_world, x_pred(3), [0,0,1,0,0,0,0,0], R_accel);
    [x_pred, P_pred] = ekf_update(x_pred, P_pred, ay_world, x_pred(6), [0,0,0,0,0,1,0,0], R_accel);

    % --- MAGNETOMETER (disabled) ---
    if USE_MAG
        Bx = double(mag(k,1));  By = double(mag(k,2));
        theta_mag = atan2(-By, Bx);
        innov_mag = wrap_angle(theta_mag - x_pred(7));
        if abs(innov_mag) < deg2rad(25)
            H_mag = [0,0,0,0,0,0,1,0];
            S_mag = H_mag * P_pred * H_mag' + (0.3)^2;
            K_mag = P_pred * H_mag' / S_mag;
            x_pred = x_pred + K_mag * innov_mag;
            x_pred(7) = wrap_angle(x_pred(7));
            P_pred = (eye(8) - K_mag * H_mag) * P_pred;
        end
    end

    % --- TIME-OF-FLIGHT UPDATES ---
    tof_raw = [double(tof1(k,:)); double(tof2(k,:)); double(tof3(k,:))];

    for s_idx = 1:3
        z_tof  = tof_raw(s_idx, 1);
        status = tof_raw(s_idx, 4);

        % Strict validity checks
        if status ~= 0;                                continue; end
        if isnan(z_tof) || isinf(z_tof);              continue; end
        if z_tof < 0.05 || z_tof > 2.5;              continue; end

        % Expected measurement from ray model
        h_val = h_tof_sensor(x_pred, sensors(s_idx), arena);
        if isinf(h_val) || isnan(h_val);              continue; end

        % Jacobian and innovation
        H_tof = compute_H_tof_numerical(x_pred, sensors(s_idx), arena, 1e-6);
        S_tof = H_tof * P_pred * H_tof' + R_tof;
        innov = z_tof - h_val;

        % TIGHTER gate: 2-sigma AND absolute 0.3m cap
        % This prevents wrong-wall corrections when heading is slightly off
        gate_dist = min(2*sqrt(S_tof), 0.30);
        if abs(innov) > gate_dist;                     continue; end

        K_tof  = P_pred * H_tof' / S_tof;
        x_pred = x_pred + K_tof * innov;
        x_pred(7) = wrap_angle(x_pred(7));
        P_pred = (eye(8) - K_tof * H_tof) * P_pred;
    end

    % --- ENFORCE ARENA BOUNDS ---
    x_pred(1) = max(arena.x_min+0.05, min(arena.x_max-0.05, x_pred(1)));
    x_pred(4) = max(arena.y_min+0.05, min(arena.y_max-0.05, x_pred(4)));

    % --- CLAMP VELOCITIES ---
    x_pred(2) = max(-1.5, min(1.5, x_pred(2)));
    x_pred(5) = max(-1.5, min(1.5, x_pred(5)));
    x_pred(8) = max(-5.0, min(5.0, x_pred(8)));

    % --- SYMMETRISE COVARIANCE ---
    P_pred = 0.5*(P_pred + P_pred');

    x_state      = x_pred;
    P            = P_pred;
    X_Est(:,k)   = x_state;
    P_Est(:,:,k) = P;
end

end % end myEKF

%==========================================================================
% HELPERS
%==========================================================================
function [x_out, P_out] = ekf_update(x_in, P_in, z, h_val, H, R)
    innov = z - h_val;
    S = H * P_in * H' + R;
    K = P_in * H' / S;
    x_out = x_in + K * innov;
    P_out = (eye(length(x_in)) - K*H) * P_in;
end

function d = h_tof_sensor(x_state, sensor, arena)
    x = x_state(1);  y = x_state(4);  theta = x_state(7);
    c = cos(theta);  s = sin(theta);
    xs = x + sensor.offset_x*c - sensor.offset_y*s;
    ys = y + sensor.offset_x*s + sensor.offset_y*c;
    ra = theta + sensor.angle_offset;
    rdx = cos(ra);  rdy = sin(ra);
    eps = 1e-6;  dists = [];

    if abs(rdx) > eps
        t = (arena.x_max - xs)/rdx;
        if t>eps && inrange(ys+t*rdy, arena.y_min, arena.y_max, eps); dists(end+1)=t; end
        t = (arena.x_min - xs)/rdx;
        if t>eps && inrange(ys+t*rdy, arena.y_min, arena.y_max, eps); dists(end+1)=t; end
    end
    if abs(rdy) > eps
        t = (arena.y_max - ys)/rdy;
        if t>eps && inrange(xs+t*rdx, arena.x_min, arena.x_max, eps); dists(end+1)=t; end
        t = (arena.y_min - ys)/rdy;
        if t>eps && inrange(xs+t*rdx, arena.x_min, arena.x_max, eps); dists(end+1)=t; end
    end
    if isempty(dists); d = Inf; else; d = min(dists); end
end

function ok = inrange(v, lo, hi, eps)
    ok = v >= lo-eps && v <= hi+eps;
end

function H = compute_H_tof_numerical(x_state, sensor, arena, ev)
    H  = zeros(1,8);
    h0 = h_tof_sensor(x_state, sensor, arena);
    if isinf(h0); return; end
    for idx = [1, 4, 7]
        xp = x_state; xp(idx) = xp(idx)+ev;
        xm = x_state; xm(idx) = xm(idx)-ev;
        hp = h_tof_sensor(xp, sensor, arena);
        hm = h_tof_sensor(xm, sensor, arena);
        if ~isinf(hp) && ~isinf(hm); H(idx) = (hp-hm)/(2*ev); end
    end
end

function a = wrap_angle(a)
    a = mod(a+pi, 2*pi) - pi;
end