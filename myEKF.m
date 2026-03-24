function [X_Est, P_Est] = myEKF(acc, gyro, mag, tof1, tof2, tof3, temp, lp_acc, gt_init)
% myEKF — Extended Kalman Filter for sub-terranean robot navigation
% COMP0217 Final Project — University College London
%
% CALLING CONVENTION:
%   First call:  myEKF(acc,gyro,mag,tof1,tof2,tof3,temp,lp_acc,[x0,y0,theta0])
%   All others:  myEKF(acc,gyro,mag,tof1,tof2,tof3,temp,lp_acc)
%
% STATE VECTOR (world frame):
%   x_state = [x, vx, ax,  y, vy, ay,  theta, omega]
%              1   2   3   4   5   6      7      8
%
% =========================================================================


persistent x_state P arena ...
    GYRO_Z_IDX GYRO_Z_SIGN gyro_bias_z R_gyro OMEGA_STILL_THRESH ...
    ACCEL_X_IDX ACCEL_X_SIGN ACCEL_BIAS_X ...
    ACCEL_Y_IDX ACCEL_Y_SIGN ACCEL_BIAS_Y ...
    sensors Q R_accel R_tof ...
    MAG_FUNCTIONAL mag_hard_iron mag_soft_iron mag_yaw_offset R_mag ...
    still_counter reacq_fired is_init sample_count ...
    prev_tof

if nargin < 9; gt_init = []; end

% =========================================================================
%  ONE-TIME SETUP — runs only on the very first call each simulation
% =========================================================================
if isempty(is_init)

    % --- Arena bounds [m] ---
    arena.x_min = -1.244;  arena.x_max = 1.244;
    arena.y_min = -1.244;  arena.y_max = 1.244;

    % -----------------------------------------------------------------
    %  GYRO
    %  Update these three values from calibrate_sensors.m output.
    % -----------------------------------------------------------------
    GYRO_Z_IDX        = 1;        % which column (1/2/3) measures yaw rate
    GYRO_Z_SIGN       = +1;        % +1 or -1
    gyro_bias_z       = 0.001883; % [rad/s] static bias
    R_gyro            = (0.005)^2;% measurement noise variance

    % OMEGA_STILL_THRESH: below this on ALL columns = truly stationary.
    % Kept as a tunable constant so re-acquisition does not fire mid-spin.
    OMEGA_STILL_THRESH = 0.08;    % [rad/s]

    % -----------------------------------------------------------------
    %  ACCELEROMETER
    %  Update from calibrate_sensors.m output.
    % -----------------------------------------------------------------
    ACCEL_X_IDX  = 3;        % forward axis column
    ACCEL_X_SIGN = +1;
    ACCEL_BIAS_X = -0.3962;  % [m/s^2]
    ACCEL_Y_IDX  = 2;        % lateral axis column
    ACCEL_Y_SIGN = +1;
    ACCEL_BIAS_Y =  0.0278;  % [m/s^2]
    R_accel      = (1.02)^2;

    % -----------------------------------------------------------------
    %  ToF SENSOR ORIENTATIONS
    %  angle_offset is relative to robot heading (theta):
    %    0       = forward
    %    +pi/2   = left
    %    -pi/2   = right
    %    pi/-pi  = backward
    %  Update from calibrate_sensors.m output.
    % -----------------------------------------------------------------
    sensors(1).angle_offset =  pi/2;   % ToF1 -> Left
    sensors(1).offset_x = 0;  sensors(1).offset_y =  0;
    sensors(2).angle_offset =  pi;     % ToF2 -> Backward
    sensors(2).offset_x = 0;  sensors(2).offset_y =  0.00;
    sensors(3).angle_offset = -pi/2;   % ToF3 -> Right
    sensors(3).offset_x = 0;  sensors(3).offset_y = 0;
    R_tof =  0.02;

    % -----------------------------------------------------------------
    %  MAGNETOMETER
    %  MAG_FUNCTIONAL — set to true only if calibrate_sensors.m confirms
    %                   the sensor has enough field range.
    %  mag_hard_iron  — [Mx_offset, My_offset] in Tesla
    %  mag_soft_iron  — [Mx_scale,  My_scale]  in Tesla (semi-axis lengths)
    %  mag_yaw_offset — heading offset [rad] to align mag axis with robot fwd
    %  R_mag          — heading measurement noise variance [rad^2]
    %
    %  How it is used:
    %    Mx_cal = (mag(1) - mag_hard_iron(1)) / mag_soft_iron(1)
    %    My_cal = (mag(2) - mag_hard_iron(2)) / mag_soft_iron(2)
    %    z_mag  = atan2(My_cal, Mx_cal) + mag_yaw_offset
    %    => soft EKF update on state index 7 (theta)
    %
    %  The update is gated: it only fires when omega < MAG_UPDATE_OMEGA_MAX
    %  so it does not try to correct heading mid-spin.
    %
    %  Update these values from calibrate_sensors.m output.
    % -----------------------------------------------------------------
    MAG_FUNCTIONAL   = false;    % disabled: 78 deg std makes it harmful
    mag_hard_iron    = [1.470000e-05, -4.020000e-05];  % [Mx_offset, My_offset] T
    mag_soft_iron    = [2.340000e-05, 2.370000e-05];   % [Mx_scale,  My_scale]  T
    mag_yaw_offset   = -0.1017;              % [rad] body->world heading align
    R_mag            = 1.8646;               % heading noise var [rad^2]

    % -----------------------------------------------------------------
    %  PROCESS NOISE COVARIANCE Q
    %  Larger values = trust the motion model less (accel noise dominates).
    % -----------------------------------------------------------------
    Q = zeros(8,8);
    Q(1,1) = 0.001;    % x position
    Q(2,2) = 0.05;     % x velocity (higher = adapts faster to motion)
    Q(3,3) = 3;        % x acceleration (large: accel model is rough)
    Q(4,4) = 0.001;    % y position
    Q(5,5) = 0.05;     % y velocity
    Q(6,6) = 3;        % y acceleration
    Q(7,7) = 0.01;     % heading
    Q(8,8) = 0.01;     % angular rate

    % -----------------------------------------------------------------
    %  INITIAL STATE & COVARIANCE
    % -----------------------------------------------------------------
    x_state       = zeros(8,1);
    P             = diag([0.5, 1.0, 4.0, 0.5, 1.0, 4.0, 0.5, 1.0]);
    still_counter = 0;
    reacq_fired   = false;
    sample_count  = 0;
    prev_tof      = nan(1,3);   % track previous ToF readings to skip duplicates
    is_init       = true;
end

sample_count = sample_count + 1;
dt = 1 / 200;   % All signals logged at 200 Hz in Simulink

% =========================================================================
%  FIRST-CALL INITIALISATION
%  Set position from ground-truth start, scan heading from ToF.
% =========================================================================
if sample_count == 1 && ~isempty(gt_init) && numel(gt_init) >= 3
    x_state(1) = gt_init(1);   % x [m]
    x_state(4) = gt_init(2);   % y [m]
    P(1,1) = 0.005;
    P(4,4) = 0.005;

    % Full 360-candidate heading scan at startup
    tof_z      = read_tof(tof1, tof2, tof3);
    x_state(7) = heading_scan_full(gt_init(1), gt_init(2), tof_z, ...
                                    gt_init(3), sensors, arena);
    P(7,7) = (5*pi/180)^2;

    fprintf('=== EKF INIT ===\n');
    fprintf('  pos=(%.3f, %.3f)  theta_quat=%.1f deg  theta_scan=%.1f deg\n', ...
        gt_init(1), gt_init(2), rad2deg(gt_init(3)), rad2deg(x_state(7)));
    fprintf('  tof=[%.3f %.3f %.3f]\n', tof_z(1), tof_z(2), tof_z(3));
    fprintf('  MAG_FUNCTIONAL=%d\n', MAG_FUNCTIONAL);
    fprintf('================\n');
end

% =========================================================================
%  PREDICTION STEP  (constant-acceleration kinematic model)
% =========================================================================
% Velocity damping factor: models friction / drag so velocity decays
% when acceleration drops.  alpha=0.98 means ~2% decay per 5ms step.
alpha_v = 0.95;   % velocity damping per step
F = [1, dt, 0.5*dt^2, 0,  0,        0, 0,   0;
     0,  alpha_v,  dt, 0,  0,        0, 0,   0;
     0,  0,        1, 0,  0,        0, 0,   0;
     0,  0,        0, 1, dt, 0.5*dt^2, 0,   0;
     0,  0,        0, 0, alpha_v,  dt, 0,   0;
     0,  0,        0, 0,  0,        1, 0,   0;
     0,  0,        0, 0,  0,        0, 1,  dt;
     0,  0,        0, 0,  0,        0, 0,   1];

x_pred = F * x_state;
P_pred = F * P * F' + Q;

% =========================================================================
%  GYRO UPDATE
%  Measures angular rate (state index 8).
% =========================================================================
omega_for_gate = 0;   % used later for motion gating
if ~any(isnan(gyro))
    omega_raw   = GYRO_Z_SIGN * double(gyro(GYRO_Z_IDX));
    omega_meas  = omega_raw - gyro_bias_z;
    H_gyro      = [0,0,0, 0,0,0, 0,1];
    [x_pred, P_pred] = ekf_update(x_pred, P_pred, omega_meas, x_pred(8), ...
                                   H_gyro, R_gyro);

    % Use max of all columns for motion gating — guards against axes where
    % one column reads near-zero during actual yaw rotations.
    omega_for_gate = max(abs(double(gyro)));

    % Per-second diagnostic log
    if mod(sample_count, 200) == 0
        fprintf('[t=%5.1fs] gyro_raw=[%+.4f %+.4f %+.4f]  omega=%.4f  theta=%+.1f deg\n', ...
            sample_count/200, gyro(1), gyro(2), gyro(3), omega_meas, rad2deg(x_pred(7)));
    end
end

% =========================================================================
%  MAGNETOMETER UPDATE
%
%  Provides an absolute heading reference to correct slow gyro drift.
%
%  Measurement model:
%    Mx_cal = (mag(1) - hi_x) / si_x          (hard + soft iron correction)
%    My_cal = (mag(2) - hi_y) / si_y
%    z_mag  = atan2(My_cal, Mx_cal) + yaw_offset
%
%  The innovation angle is always wrapped to (-pi, pi].
%
%  GATING:
%    - MAG_FUNCTIONAL must be true (from calibration)
%    - Valid (non-NaN) mag reading
%    - Robot is approximately still (|omega| < MAG_UPDATE_OMEGA_MAX)
%      This prevents spurious heading corrections mid-rotation.
% =========================================================================
MAG_UPDATE_OMEGA_MAX = 0.15;   % [rad/s] — tune if mag causes jitter

if MAG_FUNCTIONAL && ~any(isnan(mag)) && omega_for_gate < MAG_UPDATE_OMEGA_MAX
    Mx = double(mag(1));
    My = double(mag(2));

    % Hard-iron then soft-iron correction
    Mx_cal = (Mx - mag_hard_iron(1)) / mag_soft_iron(1);
    My_cal = (My - mag_hard_iron(2)) / mag_soft_iron(2);

    % Derive heading from horizontal mag field
    z_mag = wrap_angle(atan2(My_cal, Mx_cal) + mag_yaw_offset);

    % Innovation — must be angle-wrapped to avoid 2pi jumps
    innov_mag = wrap_angle(z_mag - x_pred(7));

    % EKF update manually (standard linear update, but wrapping innovation)
    H_mag = [0,0,0, 0,0,0, 1,0];
    S_mag = H_mag * P_pred * H_mag' + R_mag;
    K_mag = P_pred * H_mag' / S_mag;
    x_pred = x_pred + K_mag * innov_mag;
    x_pred(7) = wrap_angle(x_pred(7));
    P_pred = (eye(8) - K_mag * H_mag) * P_pred;
end

% =========================================================================
%  ACCELEROMETER UPDATE
%  Body-frame accel → world frame via current heading, then updates ax/ay.
% =========================================================================
if ~any(isnan(acc))
    ax_body  = ACCEL_X_SIGN * (double(acc(ACCEL_X_IDX)) - ACCEL_BIAS_X);
    ay_body  = ACCEL_Y_SIGN * (double(acc(ACCEL_Y_IDX)) - ACCEL_BIAS_Y);
    c = cos(x_pred(7));  s = sin(x_pred(7));
    ax_world =  c*ax_body - s*ay_body;
    ay_world =  s*ax_body + c*ay_body;
    [x_pred, P_pred] = ekf_update(x_pred, P_pred, ax_world, x_pred(3), ...
                                   [0,0,1, 0,0,0, 0,0], R_accel);
    [x_pred, P_pred] = ekf_update(x_pred, P_pred, ay_world, x_pred(6), ...
                                   [0,0,0, 0,0,1, 0,0], R_accel);
end

% =========================================================================
%  ToF UPDATES
%
%  Corrects position (x, y) ONLY.  H_tof(7) and H_tof(8) are zeroed so
%  ToF never directly corrects heading.  Heading is gyro's job; ToF's job
%  is position.  Mixing them causes heading/position feedback loops.
%
%  GATING:
%    - Valid reading: status=0, range in [0.05, 2.5] m
%    - Not spinning fast (|omega| <= 1.0 rad/s)
%    - Innovation within 3-sigma gate
% =========================================================================
tof_inputs = {tof1, tof2, tof3};
for s_idx = 1:3
    td = tof_inputs{s_idx};
    if any(isnan(td));               continue; end
    if td(4) ~= 0;                   continue; end
    z_tof = double(td(1));
    if isnan(z_tof) || isinf(z_tof); continue; end
    if z_tof < 0.05 || z_tof > 2.5; continue; end
    if abs(x_pred(8)) > 0.3;         continue; end   % skip during turns (wall holes)
    % Skip duplicate readings (same value = zero-order hold, not new data)
    if z_tof == prev_tof(s_idx);     continue; end
    prev_tof(s_idx) = z_tof;
    % Signal-strength filter: low signal = through wall hole or bad reading
    sig = double(td(3));
    if sig < 300;                     continue; end

    h_val = h_tof_sensor(x_pred, sensors(s_idx), arena);
    if isinf(h_val) || isnan(h_val); continue; end

    H_tof    = compute_H_tof_numerical(x_pred, sensors(s_idx), arena, 1e-6);
    H_tof(7) = 0;   % heading correction: gyro only, not ToF
    H_tof(8) = 0;

    S_tof = H_tof * P_pred * H_tof' + R_tof;
    innov = z_tof - h_val;

    gate = min(3*sqrt(S_tof), 0.8);   % 3-sigma, cap at 0.8 m
    if abs(innov) > gate;              continue; end

    K_tof  = P_pred * H_tof' / S_tof;
    x_pred = x_pred + K_tof * innov;
    x_pred(7) = wrap_angle(x_pred(7));
    P_pred = (eye(8) - K_tof*H_tof) * P_pred;
end

% =========================================================================
%  POST-ROTATION HEADING RE-ACQUISITION
%
%  After a rotation event (omega was large, now settled), run a single
%  constrained heading scan over ToF readings.  This corrects any gyro
%  accumulation during the rotation.  Fires once per rotation event.
%
%  Uses max(|gyro|) across all columns so it does not mis-fire on axes
%  that read near-zero during actual yaw rotations.
% =========================================================================
% REACQ disabled — gyro handles heading, ToF handles position only.
% Brute-force heading scans cause harmful jumps when position is uncertain.
if omega_for_gate > OMEGA_STILL_THRESH
    still_counter = 0;
    reacq_fired   = false;
else
    still_counter = still_counter + 1;
end

% =========================================================================
%  ZERO-VELOCITY UPDATE (ZUPT)
%
%  When all gyro axes are quiet AND acceleration is near its stationary
%  value, the robot is not moving.  Inject vx=0, vy=0 measurements to
%  kill residual velocity from the motion model.
% =========================================================================
ZUPT_R = 0.001;  % velocity measurement noise [m/s]^2
ZAUT_R = 0.1;    % acceleration measurement noise [m/s^2]^2
if omega_for_gate < OMEGA_STILL_THRESH
    % Also check accel is near stationary (col1 ≈ gravity, col2/col3 near bias)
    if ~any(isnan(acc))
        accel_dev = abs(double(acc(1)) - 10.0);  % col1 should be ~10 m/s² (gravity)
        if accel_dev < 0.5  % within 0.5 m/s² of gravity → stationary
            % Zero-velocity update
            [x_pred, P_pred] = ekf_update(x_pred, P_pred, 0, x_pred(2), ...
                                           [0,1,0, 0,0,0, 0,0], ZUPT_R);
            [x_pred, P_pred] = ekf_update(x_pred, P_pred, 0, x_pred(5), ...
                                           [0,0,0, 0,1,0, 0,0], ZUPT_R);
            % Zero-acceleration update
            [x_pred, P_pred] = ekf_update(x_pred, P_pred, 0, x_pred(3), ...
                                           [0,0,1, 0,0,0, 0,0], ZAUT_R);
            [x_pred, P_pred] = ekf_update(x_pred, P_pred, 0, x_pred(6), ...
                                           [0,0,0, 0,0,1, 0,0], ZAUT_R);
        end
    end
end

% =========================================================================
%  STATE / COVARIANCE CLIPPING
% =========================================================================
x_pred(1) = max(arena.x_min+0.05, min(arena.x_max-0.05, x_pred(1)));
x_pred(4) = max(arena.y_min+0.05, min(arena.y_max-0.05, x_pred(4)));
x_pred(2) = max(-1.5, min(1.5, x_pred(2)));
x_pred(5) = max(-1.5, min(1.5, x_pred(5)));
x_pred(8) = max(-5.0, min(5.0, x_pred(8)));
x_pred(7) = wrap_angle(x_pred(7));
P_pred    = 0.5*(P_pred + P_pred');   % enforce symmetry

x_state = x_pred;
P       = P_pred;
X_Est   = x_state;
P_Est   = P;

end  % ← end myEKF


% =========================================================================
%  HEADING SCAN HELPERS
% =========================================================================

function best_theta = heading_scan_full(px, py, tof_z, theta_fallback, sensors, arena)
    % Two-pass heading scan: coarse (1 deg) then fine (0.05 deg) around best.
    % Pass 1: coarse scan
    N_coarse = 360;
    theta_coarse = linspace(-pi, pi, N_coarse+1);
    theta_coarse = theta_coarse(1:N_coarse);
    best_theta  = theta_fallback;
    best_n      = -1;
    best_sse    = Inf;
    for tc = 1:N_coarse
        th     = theta_coarse(tc);
        x_cand = zeros(8,1);
        x_cand(1) = px;  x_cand(4) = py;  x_cand(7) = th;
        n = 0;  sse = 0;
        for si = 1:3
            if isnan(tof_z(si)); continue; end
            h = h_tof_sensor(x_cand, sensors(si), arena);
            if isinf(h) || isnan(h); continue; end
            innov = tof_z(si) - h;
            if abs(innov) < 0.5
                n = n+1;  sse = sse+innov^2;
            end
        end
        if n > best_n || (n == best_n && sse < best_sse)
            best_n = n;  best_sse = sse;  best_theta = th;
        end
    end
    % Pass 2: fine scan ±3 deg around coarse best (0.05 deg steps)
    fine_range = deg2rad(3);
    N_fine = 120;
    theta_fine = linspace(best_theta - fine_range, best_theta + fine_range, N_fine+1);
    for tc = 1:N_fine+1
        th     = theta_fine(tc);
        x_cand = zeros(8,1);
        x_cand(1) = px;  x_cand(4) = py;  x_cand(7) = th;
        n = 0;  sse = 0;
        for si = 1:3
            if isnan(tof_z(si)); continue; end
            h = h_tof_sensor(x_cand, sensors(si), arena);
            if isinf(h) || isnan(h); continue; end
            innov = tof_z(si) - h;
            if abs(innov) < 0.5
                n = n+1;  sse = sse+innov^2;
            end
        end
        if n > best_n || (n == best_n && sse < best_sse)
            best_n = n;  best_sse = sse;  best_theta = th;
        end
    end
    best_theta = wrap_angle(best_theta);
end

function theta_out = heading_scan_constrained(px, py, tof_z, theta_current, ...
                                               sensors, arena, max_dev, min_ratio)
    % Constrained scan — only considers headings within max_dev of current.
    % Accepts new heading only if SSE improves by factor min_ratio.
    N_cands = 360;
    theta_cands = linspace(-pi, pi, N_cands+1);
    theta_cands = theta_cands(1:N_cands);
    best_theta  = theta_current;
    best_score  = Inf;

    x_cur        = zeros(8,1);
    x_cur(1) = px;  x_cur(4) = py;  x_cur(7) = theta_current;
    baseline = tof_sse(x_cur, tof_z, sensors, arena);

    for tc = 1:N_cands
        th = theta_cands(tc);
        if abs(wrap_angle(th - theta_current)) > max_dev; continue; end
        x_cand    = zeros(8,1);
        x_cand(1) = px;  x_cand(4) = py;  x_cand(7) = th;
        s = tof_sse(x_cand, tof_z, sensors, arena);
        if s < best_score;  best_score = s;  best_theta = th;  end
    end

    if isfinite(best_score) && best_score < baseline / min_ratio
        theta_out = wrap_angle(best_theta);
    else
        theta_out = theta_current;
    end
end

function s = tof_sse(x8, tof_z, sensors, arena)
    n = 0;  sse = 0;
    for si = 1:3
        if isnan(tof_z(si)); continue; end
        h = h_tof_sensor(x8, sensors(si), arena);
        if isinf(h) || isnan(h); continue; end
        innov = tof_z(si) - h;
        if abs(innov) < 0.5;  n = n+1;  sse = sse+innov^2;  end
    end
    if n == 0;  s = Inf;  else;  s = sse/n;  end
end

function tof_z = read_tof(tof1, tof2, tof3)
    % Return [d1, d2, d3]; NaN if reading invalid.
    tof_z = nan(1,3);
    raw   = {tof1, tof2, tof3};
    for si = 1:3
        td = raw{si};
        if ~any(isnan(td)) && td(4)==0 && td(1)>=0.05 && td(1)<=2.5
            tof_z(si) = double(td(1));
        end
    end
end


% =========================================================================
%  EKF CORE HELPERS
% =========================================================================

function [x_out, P_out] = ekf_update(x_in, P_in, z, h_val, H, R)
    % Standard linear EKF update.
    innov = z - h_val;
    S     = H * P_in * H' + R;
    K     = P_in * H' / S;
    x_out = x_in + K * innov;
    P_out = (eye(length(x_in)) - K*H) * P_in;
end

function d = h_tof_sensor(x_state, sensor, arena)
    % Predicted distance from (sensor position) in (sensor direction) to wall.
    x = x_state(1);  y = x_state(4);  theta = x_state(7);
    c = cos(theta);  s = sin(theta);
    % Sensor position in world frame (accounting for body-frame offset)
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
    if isempty(dists);  d = Inf;  else;  d = min(dists);  end
end

function ok = inrange(v, lo, hi, eps)
    ok = v >= lo-eps && v <= hi+eps;
end

function H = compute_H_tof_numerical(x_state, sensor, arena, ev)
    % Numerical Jacobian for ToF measurement.
    H  = zeros(1,8);
    h0 = h_tof_sensor(x_state, sensor, arena);
    if isinf(h0);  return;  end
    for idx = [1, 4, 7]    % only x, y, theta matter
        xp = x_state;  xp(idx) = xp(idx) + ev;
        xm = x_state;  xm(idx) = xm(idx) - ev;
        hp = h_tof_sensor(xp, sensor, arena);
        hm = h_tof_sensor(xm, sensor, arena);
        if ~isinf(hp) && ~isinf(hm)
            H(idx) = (hp - hm) / (2*ev);
        end
    end
end

function a = wrap_angle(a)
    a = mod(a + pi, 2*pi) - pi;
end