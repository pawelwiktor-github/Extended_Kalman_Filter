% Extended Kalman Filter implementation

clear; clc;
load 'pos_vel_data.mat'

% Constants definition
Svk = [9e-06 * eye(3), zeros(3); zeros(3), eye(3) * 1e-8]; % Output noise
SPTu = 100; % Number of integration steps per unit time
C = [eye(3), zeros(3); zeros(3), eye(3)]; % State output matrix

m0 = [y(1,:), v(1,:)]; % Initial conditions for state variables
S0 = Svk; % Initial condition for covariance matrix

% Transfer of matrix to vector
x0 = matrix_to_vector(m0, S0);

%% EKF main loop

for i = 2:length(t)
    % Integration time constant
    tf = t(i)-t(i-1);

    % Numerical integration RK4 / Prediction
    [tt,x] = rk4(x0,tf,SPTu);

    % Save last estimate
    x_last = x(end, :)';

    % PTransfer of vector to matrix 
    [m, S] = vector_to_matrix(x_last);

    if s(i) == 0
        yv = [y(i, :), v(i, :)]'; % Measurement
        % Correction
        [m_c, S_c] = correction(m, S, yv, Svk, C);
        % Update state vector after correction
        x_last = matrix_to_vector(m_c, S_c);
    end

    % Reset the current state vector for RK4 method
    x0 = x_last;

    % Results
    results(i, :) = x_last(1:6)';

    % Display of EKF progress
    if mod(i, 5000) == 0
        fprintf('Iteration %d completed. There are %d left.\n', i, length(t)-i);
    end
end

% Rewriting the first measurement
results(1, :) = [y(1,:), v(1,:)];

fprintf('EKF parameter estimation completed...\n');

%% Results validation 

% Extracted parameters
estimated_position = results(:,1:3);
estimated_velocity = results(:,4:6);
y_meas = y(1:length(estimated_position), :);
t_meas = t(1:length(estimated_position), :);
v_meas = v(1:length(estimated_velocity), :);
gps_indices = s == 0; % Gps - measurement indicator
v_plot = v(gps_indices, :);
t_plot = t(gps_indices);

fprintf('Drawing graphs...\n');

% Drawing graphs
figure()
subplot(3,1,1)
idx = 1;
plot(t_meas, y_meas(:, idx), "*")
hold on;
plot(t_meas, estimated_position(:, idx), 'r-', 'LineWidth', 2)
legend('Measured position', 'Estimate', 'Interpreter', 'latex');
title('EKF position estimation for X-axis', 'Interpreter', 'latex');
xlabel('Time (s)', 'Interpreter', 'latex');
ylabel('Value (km)', 'Interpreter', 'latex');
hold off;
subplot(3,1,2)
idx = 2;
plot(t_meas, y_meas(:, idx), "*")
hold on;
plot(t_meas, estimated_position(:, idx), 'r-', 'LineWidth', 2)
legend('Measured position', 'Estimate', 'Interpreter', 'latex');
title('EKF position estimation for Y-axis', 'Interpreter', 'latex');
xlabel('Time (s)', 'Interpreter', 'latex');
ylabel('Value (km)', 'Interpreter', 'latex');
hold off;
subplot(3,1,3)
idx = 3;
plot(t_meas, y_meas(:, idx), "*")
hold on;
plot(t_meas, estimated_position(:, idx), 'r-', 'LineWidth', 2)
legend('Measured position', 'Estimate', 'Interpreter', 'latex');
title('EKF position estimation for Z-axis', 'Interpreter', 'latex');
xlabel('Time (s)', 'Interpreter', 'latex');
ylabel('Value (km)', 'Interpreter', 'latex');
hold off;

figure()
subplot(3,1,1)
idx = 1;
plot(t_plot, v_plot(:, idx), "*")
hold on;
plot(t_meas, estimated_velocity(:, idx), 'g-', 'LineWidth', 2)
legend('Measured velocity', 'Estimate', 'Interpreter', 'latex');
title('EKF velocity estimation for X-axis', 'Interpreter', 'latex');
xlabel('Time (s)', 'Interpreter', 'latex');
ylabel('Value (km/s)', 'Interpreter', 'latex');
hold off;
subplot(3,1,2)
idx = 2;
plot(t_plot, v_plot(:, idx), "*")
hold on;
plot(t_meas, estimated_velocity(:, idx), 'g-', 'LineWidth', 2)
legend('Measured velocity', 'Estimate', 'Interpreter', 'latex');
title('EKF velocity estimation for X-axis', 'Interpreter', 'latex');
xlabel('Time (s)', 'Interpreter', 'latex');
ylabel('Value (km/s)', 'Interpreter', 'latex');
hold off;
subplot(3,1,3)
idx = 3;
plot(t_plot, v_plot(:, idx), "*")
hold on;
plot(t_meas, estimated_velocity(:, idx), 'g-', 'LineWidth', 2)
legend('Measured velocity', 'Estimate', 'Interpreter', 'latex');
title('EKF velocity estimation for X-axis', 'Interpreter', 'latex');
xlabel('Time (s)', 'Interpreter', 'latex');
ylabel('Value (km/s)', 'Interpreter', 'latex');
hold off;

%% Filter assessment - quality indicators

fprintf('Calculation of quality indicators...\n');
N = length(estimated_position); % Number of estimated parameters
s_calc = s(1:length(estimated_position)); % Measurement indicator

% Position
position_diff = (estimated_position - y_meas); % Difference
position_diff = abs(position_diff); % Module of difference
position_sigma = (1/N)*sum(s_calc.*(position_diff.^2)); % Standard deviation^2
position_sigma = sqrt(position_sigma); % Standard deviation

% Velocity
velocity_diff = (estimated_velocity - v_meas); % Difference
velocity_diff = abs(velocity_diff); % Module of difference
velocity_sigma = (1/N)*sum(s_calc.*(velocity_diff.^2)); % Standard deviation^2
velocity_sigma = sqrt(velocity_sigma); % Standard deviation

%Sum of differences
for i = 1:length(results)
    position_diff_sum(i) = (position_diff(i, 1)+position_diff(i, 2)+position_diff(i, 3))/3;
    velocity_diff_sum(i) = (velocity_diff(i, 1)+velocity_diff(i, 2)+velocity_diff(i, 3))/3;
end

% Averages
position_sigma_avg = mean(position_sigma);
velocity_sigma_avg = mean(velocity_sigma);

% Results display
fprintf('Standard deviation of the position estimate for the X axis: %.8f\n', position_sigma(1));
fprintf('Standard deviation of the position estimate for the Y axis: %.8f\n', position_sigma(2));
fprintf('Standard deviation of the position estimate for the Z axis: %.8f\n', position_sigma(3));
fprintf('Average standard deviation of the position estimate: %.8f\n', position_sigma_avg);
fprintf('Standard deviation of the velocity estimate for the X axis: %.8f\n', velocity_sigma(1));
fprintf('Standard deviation of the velocity estimate for the Y axis: %.8f\n', velocity_sigma(2));
fprintf('Standard deviation of the velocity estimate for the Z axis: %.8f\n', velocity_sigma(3));
fprintf('Average standard deviation of the velocity estimate: %.8f\n', velocity_sigma_avg);

% Graphs
figure()
subplot(3,1,1)
idx = 1;
plot(t_meas, position_diff(:, idx), '-r')
xlabel('Time (s)', 'Interpreter', 'latex')
ylabel('Value (km)', 'Interpreter', 'latex')
legend('Estimation error', 'Interpreter', 'latex');
title('Difference of position estimate from measured value for X-axis', 'Interpreter', 'latex');
subplot(3,1,2)
idx = 2;
plot(t_meas, position_diff(:, idx), '-r')
xlabel('Time (s)', 'Interpreter', 'latex')
ylabel('Value (km)', 'Interpreter', 'latex')
legend('Estimation error', 'Interpreter', 'latex');
title('Difference of position estimate from measured value for Y-axis', 'Interpreter', 'latex');
subplot(3,1,3)
idx = 3;
plot(t_meas, position_diff(:, idx), '-r')
xlabel('Time (s)', 'Interpreter', 'latex')
ylabel('Value (km)', 'Interpreter', 'latex')
legend('Estimation error', 'Interpreter', 'latex');
title('Difference of position estimate from measured value for Z-axis', 'Interpreter', 'latex');

figure()
subplot(3,1,1)
idx = 1;
plot(t_meas, velocity_diff(:, idx), '-g')
xlabel('Time (s)', 'Interpreter', 'latex')
ylabel('Value (km/s)', 'Interpreter', 'latex')
legend('Estimation error', 'Interpreter', 'latex');
title('Difference of velocity estimate from measured value for X-axis', 'Interpreter', 'latex');
subplot(3,1,2)
idx = 2;
plot(t_meas, velocity_diff(:, idx), '-g')
xlabel('Time (s)', 'Interpreter', 'latex')
ylabel('Value (km/s)', 'Interpreter', 'latex')
legend('Estimation error', 'Interpreter', 'latex');
title('Difference of velocity estimate from measured value for Y-axis', 'Interpreter', 'latex');
subplot(3,1,3)
idx = 3;
plot(t_meas, velocity_diff(:, idx), '-g')
xlabel('Time (s)', 'Interpreter', 'latex')
ylabel('Value (km/s)', 'Interpreter', 'latex')
legend('Estimation error', 'Interpreter', 'latex');
title('Difference of velocity estimate from measured value for Z-axis', 'Interpreter', 'latex');

figure()
subplot(2,1,1)
plot(t_meas, position_diff_sum, '-r')
xlabel('Time (s)', 'Interpreter', 'latex')
ylabel('Value (km)', 'Interpreter', 'latex')
legend('Estimation error', 'Interpreter', 'latex');
title('Average difference of position estimate from measured value', 'Interpreter', 'latex');
subplot(2,1,2)
plot(t_meas, velocity_diff_sum, '-g')
xlabel('Time (s)', 'Interpreter', 'latex')
ylabel('Value (km/s)', 'Interpreter', 'latex')
legend('Estimation error', 'Interpreter', 'latex');
title('Average difference of velocity estimate from measured value', 'Interpreter', 'latex');
