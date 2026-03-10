% TIME-OF-FLIGHT MEASUREMENT MODEL: COMPLETE MATHEMATICS
% 
% KEY POINT: State x = [x, vx, ax, y, vy, ay, θ, ω] is ALL IN WORLD FRAME
% Raw sensor measurements are in ROBOT FRAME
% We must handle frame conversions carefully

%==========================================================================
% MATHEMATICAL DERIVATION
%==========================================================================

% STATE VECTOR (World Frame)
% x_state = [x, vx, ax, y, vy, ay, θ, ω]
%            1  2   3  4  5   6  7  8
%
% x, vx, ax:    Position, velocity, acceleration in WORLD frame (x direction)
% y, vy, ay:    Position, velocity, acceleration in WORLD frame (y direction)
% θ:            Orientation (yaw angle) in WORLD frame
% ω:            Angular velocity in WORLD frame

%==========================================================================
% PART 1: SENSOR POSITION TRANSFORMATION
%==========================================================================

% PROBLEM: 
% - Robot center is at (x, y) in WORLD frame
% - Sensor is mounted at offset (dx, dy) in ROBOT (BODY) FRAME
% - Need to find sensor position in WORLD frame
%
% SOLUTION: Apply rotation matrix

% Rotation Matrix (Body → World):
% The matrix that converts a vector from body frame to world frame
%
%     R(θ) = [cos(θ)  -sin(θ)]
%            [sin(θ)   cos(θ)]
%
% Physical interpretation:
%   - First column: where body x-axis points in world frame
%   - Second column: where body y-axis points in world frame

% Sensor Position Transformation:
%
%     [x_sensor]       [x]       [cos(θ)  -sin(θ)] [dx]
%     [y_sensor]  =    [y]  +    [sin(θ)   cos(θ)] [dy]
%
% Expanded (multiply out the matrix):
%
%     x_sensor = x + dx*cos(θ) - dy*sin(θ)
%     y_sensor = y + dx*sin(θ) + dy*cos(θ)

% WHY THIS FORM?
% - cos(θ), sin(θ) are the world-frame components of the body-frame x-axis
% - -sin(θ), cos(θ) are the world-frame components of the body-frame y-axis
% - So we're expressing body offset (dx, dy) in world coordinates

% EXAMPLE 1: Forward Sensor (dx=0.1, dy=0)
%
%     x_sensor = x + 0.1*cos(θ) - 0*sin(θ) = x + 0.1*cos(θ)
%     y_sensor = y + 0.1*sin(θ) + 0*cos(θ) = y + 0.1*sin(θ)
%
% If θ = 0° (facing east/+x):
%     x_sensor = x + 0.1*1 - 0 = x + 0.1    ✓ (ahead in x)
%     y_sensor = y + 0.1*0 + 0 = y          ✓ (no y offset)
%
% If θ = 90° (facing north/+y):
%     x_sensor = x + 0.1*0 - 0 = x          ✓ (no x offset)
%     y_sensor = y + 0.1*1 + 0 = y + 0.1    ✓ (ahead in y)
%
% If θ = 180° (facing west/-x):
%     x_sensor = x + 0.1*(-1) - 0 = x - 0.1 ✓ (behind in x)
%     y_sensor = y + 0.1*0 + 0 = y          ✓ (no y offset)

% EXAMPLE 2: Left Sensor (dx=0, dy=0.1)
%
%     x_sensor = x + 0*cos(θ) - 0.1*sin(θ) = x - 0.1*sin(θ)
%     y_sensor = y + 0*sin(θ) + 0.1*cos(θ) = y + 0.1*cos(θ)
%
% If θ = 0° (facing east/+x):
%     x_sensor = x - 0.1*0 = x              ✓ (no x offset)
%     y_sensor = y + 0.1*1 = y + 0.1        ✓ (left is +y)
%
% If θ = 90° (facing north/+y):
%     x_sensor = x - 0.1*1 = x - 0.1        ✓ (left is -x)
%     y_sensor = y + 0.1*0 = y              ✓ (no y offset)

% IMPLEMENTATION IN CODE:
%
%     x = x_state(1);
%     y = x_state(4);
%     theta = x_state(7);
%     
%     c = cos(theta);
%     s = sin(theta);
%     
%     x_sensor = x + dx*c - dy*s;
%     y_sensor = y + dx*s + dy*c;

%==========================================================================
% PART 2: RAY DIRECTION TRANSFORMATION
%==========================================================================

% PROBLEM:
% - Sensor points at angle angle_offset in ROBOT (BODY) FRAME
% - Need to find ray direction in WORLD frame

% SOLUTION:
% Robot's heading is θ. A sensor pointing at angle_offset in body frame
% points at angle (θ + angle_offset) in world frame

%     ray_angle = θ + angle_offset
%
% The ray direction vector (unit length):
%
%     ray_dx = cos(ray_angle) = cos(θ + angle_offset)
%     ray_dy = sin(ray_angle) = sin(θ + angle_offset)

% EXAMPLE 1: Forward Sensor (angle_offset = 0°)
%
%     ray_angle = θ + 0° = θ
%     ray_dx = cos(θ)
%     ray_dy = sin(θ)
%
% This makes sense: forward sensor points in direction θ

% EXAMPLE 2: Left Sensor (angle_offset = 90°)
%
%     ray_angle = θ + 90°
%     ray_dx = cos(θ + 90°) = -sin(θ)    [using trig identity]
%     ray_dy = sin(θ + 90°) = cos(θ)
%
% If θ = 0° (facing east/+x):
%     ray_angle = 0° + 90° = 90°
%     ray_dx = 0, ray_dy = 1             ✓ (points north/+y)

% EXAMPLE 3: Right Sensor (angle_offset = -90°)
%
%     ray_angle = θ - 90°
%     ray_dx = cos(θ - 90°) = sin(θ)
%     ray_dy = sin(θ - 90°) = -cos(θ)

% WHY THIS WORKS:
% The robot itself is rotated by angle θ. A sensor offset by angle_offset
% in the robot's frame appears at θ + angle_offset in the world frame.

% IMPLEMENTATION IN CODE:
%
%     ray_angle = theta + angle_offset;
%     ray_dx = cos(ray_angle);
%     ray_dy = sin(ray_angle);

%==========================================================================
% PART 3: PARAMETRIC RAY EQUATION
%==========================================================================

% A ray starts at (x_sensor, y_sensor) and travels in direction (ray_dx, ray_dy)
% Any point on this ray can be written as:
%
%     point(t) = (x_sensor, y_sensor) + t*(ray_dx, ray_dy)
%
% Component form:
%
%     x(t) = x_sensor + t*ray_dx
%     y(t) = y_sensor + t*ray_dy
%
% where t ≥ 0 is the DISTANCE traveled along the ray
%
% WHY t IS THE DISTANCE:
% The direction vector (ray_dx, ray_dy) has magnitude:
%     |(ray_dx, ray_dy)| = √(cos²(ray_angle) + sin²(ray_angle)) = 1
% 
% Since it's a UNIT vector, the parameter t equals distance traveled.

% EXAMPLE:
% Ray starts at (0.1, 0) pointing east (ray_dx=1, ray_dy=0)
% At t = 0.5, the ray is at:
%     x(0.5) = 0.1 + 0.5*1 = 0.6
%     y(0.5) = 0 + 0.5*0 = 0
% Distance traveled: 0.5 m ✓

%==========================================================================
% PART 4: RAY-WALL INTERSECTION (RAY CASTING)
%==========================================================================

% PROBLEM:
% Find where the ray hits a wall and at what distance

% ARENA SETUP:
% Rectangular arena with four walls:
%     Left wall:   x = x_min
%     Right wall:  x = x_max
%     Back wall:   y = y_min
%     Front wall:  y = y_max

% INTERSECTION WITH RIGHT WALL (x = x_max):
%
% Ray hits wall when x(t) = x_max
%     x_sensor + t*ray_dx = x_max
%
% Solve for t:
%     t = (x_max - x_sensor) / ray_dx
%
% CONSTRAINTS (must all be satisfied for valid intersection):
% 1. t > 0              (ray points toward wall, not away)
% 2. ray_dx ≠ 0         (ray not parallel to wall)
% 3. y_min ≤ y(t) ≤ y_max
%                       (intersection point on the wall, not past edge)
%
% Where y(t) = y_sensor + t*ray_dy = y_sensor + ((x_max - x_sensor)/ray_dx)*ray_dy

% INTERSECTION WITH LEFT WALL (x = x_min):
%
%     t = (x_min - x_sensor) / ray_dx
%
% Same constraints apply

% INTERSECTION WITH FRONT WALL (y = y_max):
%
% Ray hits when y(t) = y_max:
%     y_sensor + t*ray_dy = y_max
%
%     t = (y_max - y_sensor) / ray_dy
%
% CONSTRAINTS:
% 1. t > 0
% 2. ray_dy ≠ 0
% 3. x_min ≤ x(t) ≤ x_max

% INTERSECTION WITH BACK WALL (y = y_min):
%
%     t = (y_min - y_sensor) / ray_dy
%
% Same constraints

% FINDING NEAREST WALL:
%
% Compute t for all four walls. Keep only valid t values.
% The distance to nearest wall is:
%
%     d = min(valid_t_values)

% NUMERICAL CONSIDERATIONS:
% To avoid division by zero:
%     if |ray_dx| > ε  where ε = 1e-6
%         compute t for vertical walls
%     if |ray_dy| > ε
%         compute t for horizontal walls

%==========================================================================
% PART 5: THE h FUNCTION (MEASUREMENT FUNCTION)
%==========================================================================

% h IS THE COMPLETE ALGORITHM:
%
% h_tof(x_state, sensor_config, arena) → d_expected
%
% STEP-BY-STEP:

% Step 1: Extract state variables
%     x = x_state(1)           % x position (world)
%     y = x_state(4)           % y position (world)
%     theta = x_state(7)       % heading angle (world)
%     (velocities and accelerations are NOT used in h)

% Step 2: Extract sensor configuration
%     dx = sensor_config.offset_x       % body-frame x offset
%     dy = sensor_config.offset_y       % body-frame y offset
%     angle_offset = sensor_config.angle_offset  % body-frame angle

% Step 3: Transform sensor position to world frame
%     c = cos(theta)
%     s = sin(theta)
%     x_sensor = x + dx*c - dy*s       % equation from Part 1
%     y_sensor = y + dx*s + dy*c

% Step 4: Transform ray direction to world frame
%     ray_angle = theta + angle_offset
%     ray_dx = cos(ray_angle)          % equation from Part 2
%     ray_dy = sin(ray_angle)

% Step 5: Ray-cast against all four walls
%     distances = []
%     epsilon = 1e-6
%
%     if |ray_dx| > epsilon:
%         t = (x_max - x_sensor) / ray_dx
%         if t > 0:
%             y_hit = y_sensor + t*ray_dy
%             if y_min ≤ y_hit ≤ y_max:
%                 distances.append(t)
%
%         t = (x_min - x_sensor) / ray_dx
%         if t > 0:
%             y_hit = y_sensor + t*ray_dy
%             if y_min ≤ y_hit ≤ y_max:
%                 distances.append(t)
%
%     if |ray_dy| > epsilon:
%         t = (y_max - y_sensor) / ray_dy
%         if t > 0:
%             x_hit = x_sensor + t*ray_dx
%             if x_min ≤ x_hit ≤ x_max:
%                 distances.append(t)
%
%         t = (y_min - y_sensor) / ray_dy
%         if t > 0:
%             x_hit = x_sensor + t*ray_dx
%             if x_min ≤ x_hit ≤ x_max:
%                 distances.append(t)

% Step 6: Return minimum distance
%     if distances is empty:
%         return Inf
%     else:
%         return min(distances)

% KEY PROPERTY OF h:
% h is NONLINEAR because:
% - cos(θ) and sin(θ) appear in sensor position
% - cos(ray_angle) and sin(ray_angle) appear in ray direction
% - The "nearest wall" function is discontinuous (jumps at boundaries)
%
% This is why we use EKF (Extended KF), not linear KF

% SENSITIVITY OF h:
% h changes most when:
% - Robot moves perpendicular to nearest wall
% - Robot rotates, changing which wall is nearest
%
% h changes least when:
% - Robot moves parallel to nearest wall
% - Robot orientation stays fixed relative to wall

%==========================================================================
% PART 6: THE H MATRIX (JACOBIAN)
%==========================================================================

% H IS THE MATRIX OF PARTIAL DERIVATIVES:
%
%     H = [∂h/∂x, ∂h/∂vx, ∂h/∂ax, ∂h/∂y, ∂h/∂vy, ∂h/∂ay, ∂h/∂θ, ∂h/∂ω]
%
% Size: 1×8 (one measurement, eight state variables)

% WHICH DERIVATIVES ARE NON-ZERO?
%
% Looking at the h function:
%     h depends on: x_sensor, y_sensor, ray_dx, ray_dy, arena bounds
%
% From Part 1:
%     x_sensor = x + dx*cos(θ) - dy*sin(θ)     [depends on x, θ]
%     y_sensor = y + dx*sin(θ) + dy*cos(θ)     [depends on y, θ]
%
% From Part 2:
%     ray_dx = cos(θ + angle_offset)            [depends on θ]
%     ray_dy = sin(θ + angle_offset)            [depends on θ]
%
% Therefore h depends ONLY on: x, y, θ
% h does NOT depend on: vx, ax, vy, ay, ω
%
% Consequence:
%     ∂h/∂vx = 0
%     ∂h/∂ax = 0
%     ∂h/∂vy = 0
%     ∂h/∂ay = 0
%     ∂h/∂ω = 0
%
% Only non-zero entries:
%     ∂h/∂x
%     ∂h/∂y
%     ∂h/∂θ

% ANALYTICAL DERIVATIVES (VERY COMPLICATED):
%
% Consider the case where ray hits right wall (x = x_max):
%     h = t = (x_max - x_sensor) / ray_dx
%           = (x_max - (x + dx*cos(θ) - dy*sin(θ))) / cos(θ + angle_offset)
%
% Taking ∂h/∂x:
%     ∂h/∂x = ∂/∂x [(x_max - x - dx*cos(θ) + dy*sin(θ)) / cos(θ + angle_offset)]
%           = -1 / cos(θ + angle_offset)
%           = -1 / ray_dx
%
% This is simple! But the problem is:
% - Different walls have different formulas
% - At boundaries between regions, the "nearest wall" changes
% - The function becomes discontinuous
% - We'd need separate derivatives for each wall
% - At region boundaries, left and right limits don't match
%
% This complexity is why numerical differentiation is better

% NUMERICAL DIFFERENTIATION (SIMPLE & ROBUST):
%
% Definition of partial derivative:
%     ∂h/∂xi = lim(ε→0) [h(x + ε*ei) - h(x - ε*ei)] / (2ε)
%
% Approximation (for small ε):
%     ∂h/∂xi ≈ [h(x + ε*ei) - h(x - ε*ei)] / (2ε)
%
% Where ei is the unit vector in direction i
%
% Algorithm:
%     H = [0, 0, 0, 0, 0, 0, 0, 0]
%     h0 = h_tof(x_state, sensor_config, arena)
%
%     % Only compute for x, y, θ (others are exactly zero)
%
%     % ∂h/∂x (index 1)
%     x_pert = x_state; x_pert(1) += ε;
%     h_pert = h_tof(x_pert, sensor_config, arena)
%     H(1) = (h_pert - h0) / ε
%
%     % ∂h/∂y (index 4)
%     x_pert = x_state; x_pert(4) += ε;
%     h_pert = h_tof(x_pert, sensor_config, arena)
%     H(4) = (h_pert - h0) / ε
%
%     % ∂h/∂θ (index 7)
%     x_pert = x_state; x_pert(7) += ε;
%     h_pert = h_tof(x_pert, sensor_config, arena)
%     H(7) = (h_pert - h0) / ε

% ADVANTAGES OF NUMERICAL:
% 1. Simple to implement (call h twice)
% 2. Handles discontinuities automatically
% 3. Works for any h function (no derivation needed)
% 4. Numerically stable with small ε

% DISADVANTAGES:
% 1. Slightly less accurate than analytical
% 2. Requires 3 extra h function evaluations

% OPTIMAL ε:
% Too large: ε > 1e-4 causes truncation error
% Too small: ε < 1e-8 causes roundoff error
% Sweet spot: ε = 1e-6 or 1e-7

% WHY IT WORKS:
% At the current state, the nonlinear function h is locally well-approximated
% by its tangent plane. The slope of this plane is exactly the Jacobian H.
% So even though h is globally nonlinear, locally it's linear with slope H.

%==========================================================================
% PART 7: KALMAN FILTER MEASUREMENT UPDATE
%==========================================================================

% THE PROBLEM:
% We have:
%   - Predicted state: x_pred (from IMU prediction)
%   - Predicted covariance: P_pred
%   - Actual measurement: z_measured (from ToF sensor)
%   - Measurement noise covariance: R (sensor noise)
%
% We want to update our state estimate using this new measurement.

% THE SOLUTION: Extended Kalman Filter Measurement Update

% Step 1: COMPUTE EXPECTED MEASUREMENT
%     h_val = h_tof(x_pred, sensor_config, arena)
%
% This is "what should we measure if our state estimate is correct?"

% Step 2: COMPUTE MEASUREMENT JACOBIAN
%     H = compute_H_tof_numerical(x_pred, sensor_config, arena, 1e-6)
%
% This tells us sensitivity: how much does h change with state?

% Step 3: COMPUTE INNOVATION (MEASUREMENT RESIDUAL)
%     innovation = z_measured - h_val
%
% This is the disagreement between measurement and prediction.
% If innovation = 0, measurement matches prediction perfectly.
% Large innovation means something is wrong with our state estimate.

% Step 4: COMPUTE INNOVATION COVARIANCE
%     S = H * P_pred * H' + R
%
% This is the uncertainty in the innovation.
% It combines:
%   - H * P_pred * H'  : uncertainty in predicted measurement
%   - R                : measurement noise uncertainty
%
% Interpretation:
%     S = uncertainty_in_prediction + uncertainty_in_measurement
%
% If S is large, both are uncertain, so don't trust either
% If S is small, both are confident, so measurement is meaningful

% Step 5: COMPUTE KALMAN GAIN
%     K = P_pred * H' / S
%
% The Kalman gain determines how much to move toward the measurement.
%
% Units analysis:
%     P_pred: [state×state]
%     H': [state×measurement]
%     S: [measurement×measurement]
%     K = [state×state] * [state×measurement] / [measurement×measurement]
%       = [state×measurement]
%
% K tells us: "for each unit of innovation, move state by K units"
%
% If S is large (uncertain): K is small, trust prediction more
% If S is small (confident): K is large, trust measurement more

% Step 6: UPDATE STATE
%     x_updated = x_pred + K * innovation
%
% Move state estimate in the direction that reduces innovation.
% Amount to move determined by Kalman gain K.

% Step 7: UPDATE COVARIANCE
%     P_updated = (I - K * H) * P_pred
%
% Reduce uncertainty because we got new information.
% The factor (I - K*H) determines how much uncertainty decreases.
%
% Key property: P_updated ≤ P_pred (uncertainty never increases)

% COMPLETE MEASUREMENT UPDATE:
%
%     h_val = h_tof_sensor(x_pred, sensor_config, arena)
%     H = compute_H_tof_numerical(x_pred, sensor_config, arena, 1e-6)
%     innovation = z_measured - h_val
%     S = H * P_pred * H' + R_tof
%     K = P_pred * H' / S
%     x_updated = x_pred + K * innovation
%     P_updated = (I - K * H) * P_pred

% APPLIED TO THREE TOF SENSORS:
%
% for sensor_idx = 1:3
%     z_measured = extract_tof_measurement(sensor_idx)
%     h_val = h_tof_sensor(x_pred, sensors(sensor_idx), arena)
%     H = compute_H_tof_numerical(x_pred, sensors(sensor_idx), arena, 1e-6)
%     innovation = z_measured - h_val
%     S = H * P_pred * H' + R_tof
%     K = P_pred * H' / S
%     x_pred = x_pred + K * innovation
%     P_pred = (I - K * H) * P_pred
% end

% WHY THIS WORKS:
% The KF equations are OPTIMAL for linear systems (minimize error variance).
% The EKF extends them to nonlinear systems by using the Jacobian H
% instead of the system matrix.
% 
% At the operating point (current state), a nonlinear function is
% approximately linear with slope given by the Jacobian.
% The small error from this approximation is acceptable for most applications.

%==========================================================================
% PART 8: PUTTING IT ALL TOGETHER
%==========================================================================

% COMPLETE EKF LOOP FOR TOF MEASUREMENT:

function [x_est, P_est] = ekf_tof_update(x_pred, P_pred, z_tof, ...
                                        sensors, arena, R_tof)
    
    % Input:
    %   x_pred: predicted state [8×1]
    %   P_pred: predicted covariance [8×8]
    %   z_tof: measured distances [3×1] for three sensors
    %   sensors: sensor config array
    %   arena: arena bounds struct
    %   R_tof: measurement noise variance (scalar)
    %
    % Output:
    %   x_est: updated state [8×1]
    %   P_est: updated covariance [8×8]
    
    x_est = x_pred;
    P_est = P_pred;
    
    % Update for each of three sensors
    for sensor_idx = 1:3
        
        % Step 1: Compute expected measurement
        h_val = h_tof_sensor(x_est, sensors(sensor_idx), arena);
        
        % Step 2: Compute Jacobian
        H = compute_H_tof_numerical(x_est, sensors(sensor_idx), arena, 1e-6);
        
        % Step 3: Compute innovation
        innovation = z_tof(sensor_idx) - h_val;
        
        % Step 4: Compute innovation covariance
        S = H * P_est * H' + R_tof;
        
        % Step 5: Compute Kalman gain
        K = P_est * H' / S;
        
        % Step 6: Update state
        x_est = x_est + K * innovation;
        
        % Step 7: Update covariance
        P_est = (eye(8) - K * H) * P_est;
    end
end

%==========================================================================
% PART 9: NUMERICAL EXAMPLES
%==========================================================================

% EXAMPLE 1: ROBOT AT ORIGIN, FORWARD SENSOR
%
% Setup:
%   x_pred = [0; 0; 0; 0; 0; 0; 0; 0]    (at origin, no motion)
%   sensor: forward, at (0.1, 0), pointing angle_offset=0
%   arena: x ∈ [-1, 1], y ∈ [-0.5, 0.5]
%   z_measured = 0.85 (measured by sensor)
%
% Computation:
%   x = 0, y = 0, θ = 0
%   
%   Sensor position:
%     c = cos(0) = 1, s = sin(0) = 0
%     x_sensor = 0 + 0.1*1 - 0*0 = 0.1
%     y_sensor = 0 + 0.1*0 + 0*1 = 0
%   
%   Ray direction:
%     ray_angle = 0 + 0 = 0
%     ray_dx = cos(0) = 1
%     ray_dy = sin(0) = 0
%   
%   Ray-wall intersections:
%     Right wall (x=1): t = (1 - 0.1)/1 = 0.9 ✓
%     Front wall (y=0.5): ray_dy=0, skip
%     Back wall (y=-0.5): ray_dy=0, skip
%     Left wall (x=-1): t < 0, skip
%   
%   Expected distance:
%     h_val = 0.9
%
% Innovation:
%     innovation = 0.85 - 0.9 = -0.05
%     (measurement is 0.05m less than expected, so robot is closer to wall)
%
% Jacobian (numerical):
%     ∂h/∂x: perturb x by 1e-6
%       x_sensor increases by 1e-6
%       t increases by 1e-6
%       H(1) ≈ 1e-6/1e-6 = 1.0
%     
%     ∂h/∂y: perturb y by 1e-6
%       y_sensor increases by 1e-6
%       But ray_dy=0, so y doesn't affect intersection
%       H(4) ≈ 0
%     
%     ∂h/∂θ: perturb θ by 1e-6
%       x_sensor changes, ray direction changes
%       Small change in which wall is nearest
%       H(7) ≈ small value
%
% Update:
%     S = H * P_pred * H' + R_tof
%     K = P_pred * H' / S
%     x_est = x_pred + K * innovation

% EXAMPLE 2: ROBOT ROTATED
%
% Setup (same as Example 1, but θ = 45°):
%   x_pred = [0; 0; 0; 0; 0; 0; π/4; 0]
%
% Computation:
%   x = 0, y = 0, θ = 45° = π/4
%   
%   c = cos(45°) ≈ 0.707, s = sin(45°) ≈ 0.707
%   x_sensor = 0 + 0.1*0.707 - 0*0.707 ≈ 0.0707
%   y_sensor = 0 + 0.1*0.707 + 0*0.707 ≈ 0.0707
%   
%   ray_angle = 45° + 0° = 45°
%   ray_dx = cos(45°) ≈ 0.707
%   ray_dy = sin(45°) ≈ 0.707
%   
%   Ray intersections (need to check both walls now):
%     Right wall: t = (1 - 0.0707) / 0.707 ≈ 1.311
%     Front wall: t = (0.5 - 0.0707) / 0.707 ≈ 0.604
%   
%   h_val = min(1.311, 0.604) = 0.604
%     (front wall is nearer when pointing northeast)

%==========================================================================
% PART 10: SANITY CHECKS
%==========================================================================

% CHECK 1: h increases when robot moves away from wall
%   Robot at x=0 facing east: h ≈ 0.9 (distance to right wall)
%   Robot at x=0.1 facing east: h ≈ 0.8
%   h decreased! So ∂h/∂x should be NEGATIVE
%   
%   Physics: moving toward wall (increasing x toward x_max=1)
%   reduces distance to wall. ✓

% CHECK 2: h is continuous except at transitions
%   As robot rotates from θ=0 to θ=90, right wall transitions
%   to front wall. h should be continuous (both give same value
%   at transition point)
%   
%   At θ=0: right wall distance = x_max - x_sensor
%   At θ=90: front wall distance = y_max - y_sensor
%   Both should equal when sensor is equidistant to both walls. ✓

% CHECK 3: H entries have correct signs
%   Moving away from wall → h increases
%   So ∂h/∂(position) > 0 for dimension pointing away
%   and ∂h/∂(position) < 0 for dimension pointing toward wall ✓

% CHECK 4: Small rotation → h changes
%   If sensor points along wall, ∂h/∂θ ≈ 0 (rotation parallel to wall)
%   If sensor points perpendicular to wall, ∂h/∂θ is large
%   (rotation changes which wall is nearest) ✓

end
