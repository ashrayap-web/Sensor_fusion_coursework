function [X_Est, P_Est] = myEKF(acc, gyro, mag, tof1, tof2, tof3, temp, lp_acc)
% myEKF - Extended Kalman Filter for sub-terranean robot navigation
% COMP0217 Final Project - University College London
% State vector (all in WORLD frame):
%   x_state = [x, vx, ax, y, vy, ay, theta, omega]
%              1   2   3  4   5   6     7      8

persistent x_state P arena sensors Q R_accel R_gyro R_tof ...
           ACCEL_X_IDX ACCEL_X_SIGN ACCEL_Y_IDX ACCEL_Y_SIGN ...
           GYRO_Z_IDX GYRO_Z_SIGN GYRO_SCALE ...
           gyro_bias_z gyro_bias_std ...
           is_init prev_time sample_count

if isempty(is_init)

    %----------------------------------------------------------------------
    % ARENA CONFIGURATION
    %----------------------------------------------------------------------
    arena.x_min = -1.244;
    arena.x_max =  1.244;
    arena.y_min = -1.244;
    arena.y_max =  1.244;

    %----------------------------------------------------------------------
    % IMU AXIS REMAPPING
    %----------------------------------------------------------------------
    ACCEL_X_IDX  = 2;   ACCEL_X_SIGN =  1;   % Forward Accel
    ACCEL_Y_IDX  = 3;   ACCEL_Y_SIGN =  1;   % Lateral Accel
    GYRO_Z_IDX   = 1;   GYRO_Z_SIGN  =  1;   % Yaw Rate
    GYRO_SCALE   = pi/180;                   % Scale to rad/s after bias

    %----------------------------------------------------------------------
    % SENSOR MOUNTING — body frame offsets (metres)
    % ToF1=forward, ToF2=left, ToF3=right
    %----------------------------------------------------------------------
    sensors(1).offset_x =  0.00;  sensors(1).offset_y =  0.03;  sensors(1).angle_offset =  0.00;
    sensors(2).offset_x = -0.04;  sensors(2).offset_y =  0.03;  sensors(2).angle_offset =  pi/2;
    sensors(3).offset_x =  0.04;  sensors(3).offset_y =  0.03;  sensors(3).angle_offset =  pi;

    %----------------------------------------------------------------------
    % PROCESS NOISE
    %----------------------------------------------------------------------
    Q = zeros(8,8);
    Q(1,1) = 0.0001;  % x uncertainty
    Q(2,2) = 0.001;   % vx diffusion
    Q(3,3) = 3.0;     % ax uncertainty (large — don't trust accel for position)
    Q(4,4) = 0.0001;  % y uncertainty
    Q(5,5) = 0.001;   % vy diffusion
    Q(6,6) = 3.0;     % ay uncertainty
    Q(7,7) = 0.001; % theta
    Q(8,8) = 0.25;    % omega uncertainty

    %----------------------------------------------------------------------
    % MEASUREMENT NOISE
    %----------------------------------------------------------------------
    R_accel = (1.02)^2;    % 3× accel std from calibration (col2 std=0.34 m/s²)
    R_tof   = (0.35)^2;    % ToF accurate when valid

    % Gyro bias hardcoded from calib2_straight.mat (6240 stationary samples, 60s):
    %   col1 yaw: mean = -10.2227 deg/s × pi/180 = -0.17842 rad/s
    %   col1 std  =  0.2295 deg/s × pi/180 =  0.00401 rad/s
    gyro_bias_z   = -0.17842;   % rad/s
    gyro_bias_std =  0.00401;   % rad/s
    R_gyro = (gyro_bias_std * 2)^2;  % = (0.00802)^2 = 6.4e-5

    %----------------------------------------------------------------------
    % INITIAL STATE
    %----------------------------------------------------------------------
    x_state = zeros(8, 1);

    % Tight on position, loose on velocity/accel
    P = diag([0.005, 1.0, 4.0, ...
              0.005, 1.0, 4.0, ...
              0.005, 1.0]);

    prev_time    = 0;
    sample_count = 0;
    is_init      = true;
end

%==========================================================================
% TIMESTEP
%==========================================================================
sample_count = sample_count + 1;
dt = 1/104;   % nominal — overridden if sensor time is passed via temp(1)
%==========================================================================
% PREDICTION
%==========================================================================
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

%==========================================================================
% GYROSCOPE UPDATE
% Gyro directly measures omega — most reliable sensor
%==========================================================================
if ~any(isnan(gyro))
    omega_raw  = GYRO_Z_SIGN * double(gyro(GYRO_Z_IDX));
    omega_meas = (omega_raw - gyro_bias_z) * GYRO_SCALE;
    H_gyro = [0, 0, 0, 0, 0, 0, 0, 1];
    [x_pred, P_pred] = ekf_update(x_pred, P_pred, omega_meas, x_pred(8), H_gyro, R_gyro);
end

%==========================================================================
% ACCELEROMETER UPDATE (ax/ay only — does NOT directly affect x,y position)
% Subtract calibrated biases from calib2_straight.mat (60s stationary):
%   col2 bias = -0.3861 m/s²,  col3 bias = +0.0254 m/s²
%==========================================================================
if ~any(isnan(acc))
    ax_body = ACCEL_X_SIGN * (double(acc(ACCEL_X_IDX)) - (-0.3861));
    ay_body = ACCEL_Y_SIGN * (double(acc(ACCEL_Y_IDX)) - ( 0.0254));
    c = cos(x_pred(7));  s = sin(x_pred(7));
    ax_world =  c*ax_body - s*ay_body;
    ay_world =  s*ax_body + c*ay_body;
    [x_pred, P_pred] = ekf_update(x_pred, P_pred, ax_world, x_pred(3), [0,0,1,0,0,0,0,0], R_accel);
    [x_pred, P_pred] = ekf_update(x_pred, P_pred, ay_world, x_pred(6), [0,0,0,0,0,1,0,0], R_accel);
end

%==========================================================================
% TIME-OF-FLIGHT UPDATES
%==========================================================================
tof_inputs = {tof1, tof2, tof3};

for s_idx = 1:3
    t_data = tof_inputs{s_idx};

    if any(isnan(t_data));              continue; end
    if t_data(4) ~= 0;                 continue; end   % status must be OK

    z_tof = double(t_data(1));

    if isnan(z_tof) || isinf(z_tof);  continue; end
    if z_tof < 0.05 || z_tof > 2.5;  continue; end   % range validity

    % Expected measurement from ray model
    h_val = h_tof_sensor(x_pred, sensors(s_idx), arena);
    if isinf(h_val) || isnan(h_val);  continue; end

    % Jacobian and innovation
    H_tof = compute_H_tof_numerical(x_pred, sensors(s_idx), arena, 1e-6);
    S_tof = H_tof * P_pred * H_tof' + R_tof;
    innov = z_tof - h_val;

    % Gated update: 2-sigma AND absolute 0.60m cap
    gate_dist = min(2*sqrt(S_tof), 0.60);
    if abs(innov) > gate_dist;         continue; end

    K_tof  = P_pred * H_tof' / S_tof;
    x_pred = x_pred + K_tof * innov;
    x_pred(7) = wrap_angle(x_pred(7));
    P_pred = (eye(8) - K_tof * H_tof) * P_pred;
end

%==========================================================================
% POST-PROCESSING
%==========================================================================
% Enforce arena bounds
x_pred(1) = max(arena.x_min+0.05, min(arena.x_max-0.05, x_pred(1)));
x_pred(4) = max(arena.y_min+0.05, min(arena.y_max-0.05, x_pred(4)));

% Clamp velocities
x_pred(2) = max(-1.5, min(1.5, x_pred(2)));
x_pred(5) = max(-1.5, min(1.5, x_pred(5)));
x_pred(8) = max(-5.0, min(5.0, x_pred(8)));

% Wrap yaw and symmetrise covariance
x_pred(7) = wrap_angle(x_pred(7));
P_pred     = 0.5*(P_pred + P_pred');

% Store state
x_state = x_pred;
P       = P_pred;

X_Est = x_state;
P_Est = P;

end  % end myEKF

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
    ra  = theta + sensor.angle_offset;
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