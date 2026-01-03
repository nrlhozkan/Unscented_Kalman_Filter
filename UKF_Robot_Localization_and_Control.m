%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Unscented Kalman Filter for Robot Localization

clear;
close all;
clc;

% Load statistics package for Octave
pkg load statistics;
% Load video package for frame capture (Octave)
pkg load video;

%% Parameters

% Time Parameters
dt = 0.1;      % Time step
T = 40;         % Total time
t = 0:dt:T;     % Time vector

% Known Landmark Positions
landmarks = [5, 10; 
             15, 5; 
             12, 17;]; % [x1, y1; x2, y2; x3, y3; ...]

% Initial true state
x_true = [2; 5; 0.3]; % [x; y; theta]

% Process and Measurement Noise Covariances
Q = 0.0001 * eye(3); % Process noise covariance
sigma_range=0.3; sigma_bearing=0.1; % Standard deviations for range and bearing
R_single = diag([sigma_range^2, sigma_bearing^2]); % Measurement noise covariance for a single [distance; angle] pair

% Number of state dimensions
n = length(x_true);

% Number of landmarks
num_landmarks = size(landmarks, 1);

% Number of measurements (distance and angle to each landmark)
m = 2 * num_landmarks; % distance and angle to each landmark

% UKF Parameters
alpha = 0.00001;      % Spread of sigma points
beta = 2;          % Optimal for Gaussian distributions
kappa = 0;         % Secondary scaling parameter
lambda = alpha^2 * (n + kappa) - n; % Composite scaling parameter

% Weights for sigma points
Wm = [lambda / (n + lambda)]; % Weights for mean
Wc = [lambda / (n + lambda) + (1 - alpha^2 + beta)]; % Weights for covariance

for i = 1:2*n 
    Wm = [Wm; 1/(2*(n + lambda))];
    Wc = [Wc; 1/(2*(n + lambda))];
end

% Initial state estimate
x_est = zeros(n, length(t)); % State estimates
P = [0.1, 0, 0; 0, 0.1, 0; 0, 0, 0.05];                  % Initial estimate covariance

% Create block diagonal measurement noise covariance (R for all landmarks)
R = zeros(m, m);
for lm = 1:num_landmarks
    R(2*lm-1:2*lm, 2*lm-1:2*lm) = R_single;
end

% Allocate space for true states and measurements
x_store = zeros(n, length(t));
x_store(:,1) = x_true;
z_store = zeros(m, length(t));

% Allocate space to store covariance matrices
P_store = zeros(n, n, length(t));

% Initial state estimate
x_est(:,1) = [2; 5; 0.3]; % Initial guess
P_store(:,:,1) = P; % Store initial covariance
wheelbase = 0.5; % Distance between front and rear axles

%Target Position
target_position = [20; 20];

% PID Controller Parameters

% For distance control (velocity)
Kp_dist = 0.7;
Ki_dist = 0.01;
Kd_dist = 0.1;
integral_dist = 0;
prev_error_dist = 0;

% For heading control (steering)
Kp_heading = 0.1;
Ki_heading = 0.002;
Kd_heading = 0.01;
integral_heading = 0;
prev_error_heading = 0;

% Tolerance for reaching target
position_tolerance = 0.05; % meters
%% Simulation and UKF Implementation

for k = 2:length(t)

    %% Simulate true system

    % PID Controller - Generate control inputs based on estimated state

    % Use the previous estimate to generate control for this time step
    x_est_prev = x_est(:,k-1); 

    % Calculate Distance to Target
    delta_x = target_position(1) - x_est_prev(1);
    delta_y = target_position(2) - x_est_prev(2);
    distance_to_target = sqrt(delta_x^2 + delta_y^2);

    % Heading Error
    desired_heading = atan2(delta_y, delta_x);
    heading_error = normalize_angle(desired_heading - x_est_prev(3));

    % Check if the car reached the target position
    if distance_to_target < position_tolerance
        % Stop the robot
        v_command = 0;
        steering_angle = 0;

        % Stop simulation if target is reached 
        reached_time = t(k);
        fprintf('Target reached at time %.2f seconds.\n', reached_time);

        break; 
    else
        % Velocity Control (Proportional to distance)
        error_dist = distance_to_target;
        integral_dist = integral_dist + error_dist * dt;
        derivative_dist = (error_dist - prev_error_dist) / dt;
        v_command = Kp_dist * error_dist + Ki_dist * integral_dist + Kd_dist * derivative_dist;
        prev_error_dist = error_dist;

        % Heading Control (PID)
        error_heading = heading_error;
        integral_heading = integral_heading + error_heading * dt;
        derivative_heading = (error_heading - prev_error_heading) / dt;
        steering_angle = Kp_heading * error_heading + Ki_heading * integral_heading + Kd_heading * derivative_heading;
        prev_error_heading = error_heading;
        % steering_angle = 0.01;
        
    end

    % Forward Euler Equation
    hdg = x_true(3);
    dist = v_command * dt;
    beta_turn = (dist/wheelbase) * tan(steering_angle);
    R_turn = wheelbase / tan(steering_angle); % Turning radius

    x_dot = [R_turn * (sin(hdg + beta_turn) - sin(hdg));
                R_turn * (-cos(hdg + beta_turn) + cos(hdg));
                beta_turn / dt];

    x_true = x_true + x_dot * dt + mvnrnd(zeros(3,1), Q)' * sqrt(dt); % Add process noise
    x_true(3) = normalize_angle(x_true(3)); % Normalize heading angle
    x_store(:,k) = x_true;

    % Measurement with noise from all landmarks
    z = zeros(m, 1);
    for lm = 1:num_landmarks
        dx = landmarks(lm,1) - x_true(1);
        dy = landmarks(lm,2) - x_true(2);
        distance = sqrt(dx^2 + dy^2);
        angle = atan2(dy, dx) - x_true(3);
        angle = normalize_angle(angle);
        
        z(2*lm-1:2*lm) = [distance; angle] + mvnrnd(zeros(2,1), R_single)' * sqrt(dt);
    end
    z_store(:,k) = z;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
    %% UKF Prediction Step
    % Generate sigma points
    [chi] = sigma_points(x_est(:,k-1), P, lambda, n);

    % Propagate sigma points through the process model
    chi_pred = zeros(n, 2*n + 1);
    for i = 1:2*n + 1
        % Process model
        x1 = chi(1,i); % x position
        x2 = chi(2,i); % y position
        x3 = chi(3,i); % heading

        dist = v_command * dt;
        beta_turn = (dist/wheelbase) * tan(steering_angle);
        R_turn = wheelbase / tan(steering_angle); % Turning radius
        x_dot = [R_turn * (sin(x3 + beta_turn) - sin(x3));
                    R_turn * (-cos(x3 + beta_turn) + cos(x3));
                    beta_turn / dt];

        x_dot(3) = normalize_angle(x_dot(3)); % Normalize heading angle
        chi_pred(:,i) = chi(:,i) + x_dot * dt; % Euler integration
    end

    % Predicted state mean
    x_pred = chi_pred * Wm;
    
    % Predicted state covariance
    P_pred = zeros(n);
    for i = 1:2*n + 1
        dx = chi_pred(:,i) - x_pred;
        P_pred = P_pred + Wc(i) * (dx * dx');
    end
    P_pred = P_pred + Q; % Add process noise covariance

    %% UKF Update Step
    % Propagate sigma points through the measurement model for all landmarks
    z_pred_sigma = zeros(m, 2*n + 1);
    for i = 1:2*n + 1
        % Measurement model: distance and angle to all landmarks
        for lm = 1:num_landmarks
            dx = landmarks(lm,1) - chi_pred(1,i);
            dy = landmarks(lm,2) - chi_pred(2,i);
            distance = sqrt(dx^2 + dy^2);
            angle = atan2(dy, dx) - chi_pred(3,i);
            angle = normalize_angle(angle);
            z_pred_sigma(2*lm-1:2*lm, i) = [distance; angle];
        end
    end

    % Predicted measurement mean
    z_pred = z_pred_sigma * Wm;
    
    % Innovation covariance
    S = zeros(m, m);
    for i = 1:2*n + 1
        dz = z_pred_sigma(:,i) - z_pred;
        S = S + Wc(i) * (dz * dz');
    end
    S = S + R; % Add measurement noise covariance

    % Cross-covariance
    Pxz = zeros(n, m);
    for i = 1:2*n + 1
        dx = chi_pred(:,i) - x_pred;
        dz = z_pred_sigma(:,i) - z_pred;
        Pxz = Pxz + Wc(i) * (dx * dz');
    end
    
    % Kalman gain
    K = Pxz / S;

    % Update state estimate
    x_est(:,k) = x_pred + K * (z - z_pred);
    
    % Update estimate covariance
    P = P_pred - K * S * K';
    
    % Store covariance for plotting
    P_store(:,:,k) = P;

end

% Clear unused preallocated space if target was reached early
x_est = x_est(:,1:k-1);
x_store = x_store(:,1:k-1);
z_store = z_store(:,1:k-1);
P_store = P_store(:,:,1:k-1);
t = t(1:k-1);

% Generate animation video (separate function)
% fprintf('\nGenerating animation video...\n');
% create_animation(x_store, x_est, P_store, t, landmarks, target_position, num_landmarks);


%% Plot Results
figure;
subplot(3,1,1);
plot(t, x_store(1,:), 'g', 'LineWidth', 2.0); hold on;
plot(t, x_est(1,:), 'k--', 'LineWidth', 2.0);
xlabel('Time (s)');
ylabel('Position X (m)');
legend('True Position X', 'Estimated Position X', 'Measured Position X');
grid on;

subplot(3,1,2);
plot(t, x_store(2,:), 'g', 'LineWidth', 2.0); hold on;
plot(t, x_est(2,:), 'k--', 'LineWidth', 2.0);
xlabel('Time (s)');
ylabel('Position Y (m)');
legend('True Position Y', 'Estimated Position Y', 'Measured Position Y');
grid on;

subplot(3,1,3);
plot(t, x_store(3,:), 'g', 'LineWidth', 2.0); hold on;
plot(t, x_est(3,:), 'k--', 'LineWidth', 2.0);
xlabel('Time (s)');
ylabel('Heading (rad)');
legend('True Heading', 'Estimated Heading');
grid on;
title('UKF State Estimation for Robot Localization');

% Save figure 1
saveas(gcf, 'UKF_State_Estimation.png');

figure;
% Plot robot trajectory and landmarks
plot(x_store(1,:), x_store(2,:), 'g', 'LineWidth', 2.0); hold on;
plot(x_est(1,:), x_est(2,:), 'k', 'LineWidth', 2.0);
plot(landmarks(:,1), landmarks(:,2), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
plot(target_position(1), target_position(2), 'bx', 'MarkerSize', 10, 'LineWidth', 2);

% Plot covariance ellipses at regular intervals
interval = 20; % Plot every time step
for k = 1:interval:length(t)
    plot_covariance_ellipse(x_est(1:2,k), P_store(1:2,1:2,k), 3, 'g');
end

xlabel('X Position (m)');
ylabel('Y Position (m)');
legend('True Trajectory', 'Estimated Trajectory', 'Landmarks');
title('Robot Trajectory and Landmarks');
grid on;
% Save figure 2
saveas(gcf, 'Robot_Trajectory_and_Landmarks.png');

