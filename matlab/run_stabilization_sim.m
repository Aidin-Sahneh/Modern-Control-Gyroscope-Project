clear; clc; close all;

% --- Controller Design (with CORRECT B matrix) ---
A = [0 1 0 0; 0 0 0 100; 0 0 0 1; 0 -100 0 0];
B = [0 0; 20 0; 0 0; 0 20]; % Corrected B matrix
P = [-1, -2, -3, -4];
K = place(A, B, P);

% --- Simulation Setup ---
tspan = [0 15];
x0 = [1; 0; pi/4; 0]; 

% --- Run the Closed-Loop Simulation ---
[t, x] = ode45(@(t,x) closed_loop_nonlinear_system(t, x, K), tspan, x0);

% --- Plot the Results ---
figure('Name', 'Stabilized System Response');
subplot(2,1,1);
plot(t, x(:,1), 'r', t, x(:,3), 'b', 'LineWidth', 1.5);
title('Stabilized System State Variables (Angles)');
xlabel('Time (s)');
ylabel('Angle (rad)');
legend('x_1 (\theta)', 'x_3 (\phi)');
grid on;

subplot(2,1,2);
plot(t, x(:,2), 'r', t, x(:,4), 'b', 'LineWidth', 1.5);
title('Stabilized System State Variables (Angular Velocities)');
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
legend('x_2 (d\theta/dt)', 'x_4 (d\phi/dt)');
grid on;