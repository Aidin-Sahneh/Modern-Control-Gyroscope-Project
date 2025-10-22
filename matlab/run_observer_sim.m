clear;
clc;
close all;

% --- Controller Design (from Q8) ---
A = [0 1 0 0; 0 0 0 100; 0 0 0 1; 0 -100 0 0];
B = [0 0; 20 0; 0 0; 0 20];
C = [1 0 0 0; 0 0 1 0];
P_controller = [-1, -2, -3, -4];
K = place(A, B, P_controller);

% --- Observer Design (from this question) ---
P_observer = [-10, -11, -12, -13];
L = place(A', C', P_observer)';

% --- Simulation Setup ---
tspan = [0 10];
x0 = [1; 0; pi/4; 0]; % Real system initial condition
x_hat0 = [0; 0; 0; 0]; % Observer starts with zero guess
xa0 = [x0; x_hat0]; % Combined 8x1 state vector [x; x_hat]

% --- Run the Full Simulation ---
[t, xa] = ode45(@(t,xa) observer_and_plant_system(t, xa, K, L, A, B, C), tspan, xa0);

% --- Analyze and Plot Results ---
x = xa(:, 1:4); % True state
x_hat = xa(:, 5:8); % Estimated state
e = x - x_hat; % Estimation error

% Plot 1: State Comparison
figure('Name', 'State vs. Estimate');
subplot(2,1,1);
plot(t, x(:,1), 'b-', t, x_hat(:,1), 'r--', 'LineWidth', 1.5);
legend('True x_1 (\theta)', 'Estimated \theta');
title('Observer Performance: State Estimation');
ylabel('Angle (rad)');
grid on;
subplot(2,1,2);
plot(t, x(:,2), 'b-', t, x_hat(:,2), 'r--', 'LineWidth', 1.5);
legend('True x_2 (d\theta/dt)', 'Estimated d\theta/dt');
xlabel('Time (s)');
ylabel('Ang. Vel. (rad/s)');
grid on;

% Plot 2: Estimation Error
figure('Name', 'Estimation Error');
plot(t, e, 'LineWidth', 1.5);
title('Estimation Error (x - \^x)');
xlabel('Time (s)');
ylabel('Error');
legend('e_1', 'e_2', 'e_3', 'e_4');
grid on;

% --- Calculate Error Norms ---
% L-infinity norm (peak error)
norm_inf = max(vecnorm(e'));
fprintf('The L-infinity norm of the estimation error is: %f\n', norm_inf);

% L2 norm (total error energy)
norm_2 = sqrt(trapz(t, vecnorm(e').^2));
fprintf('The L2 norm of the estimation error is: %f\n', norm_2);