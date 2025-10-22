clear;
clc;
close all;

% --- System and Simulation Setup ---
A = [0 1 0 0; 0 0 0 100; 0 0 0 1; 0 -100 0 0];
B = [0 0; 20 0; 0 0; 0 20];
tspan = [0 10];
x0 = [1; 0; pi/4; 0];

% --- Define the Two Sets of Desired Poles for Comparison ---
P1 = [-1, -2, -3, -4]; % Original
P3 = [-5, -6, -7, -8]; % Faster

% --- Design Controllers and Simulate ---
% Case 1: Original Poles
K1 = place(A, B, P1);
[t1, x1] = ode45(@(t,x) closed_loop_nonlinear_system(t, x, K1), tspan, x0);
u1 = -K1 * x1'; % Calculate control input u(t)

% Case 3: Faster Poles
K3 = place(A, B, P3);
[t3, x3] = ode45(@(t,x) closed_loop_nonlinear_system(t, x, K3), tspan, x0);
u3 = -K3 * x3';

% --- Plot and Compare Results ---
% Plot 1: System Performance Comparison
figure('Name', 'System Performance Comparison');
plot(t1, x1(:,1), 'b-', 'LineWidth', 1.5);
hold on;
plot(t3, x3(:,1), 'g:', 'LineWidth', 2);
title('System Performance Comparison');
xlabel('Time (s)');
ylabel('Angle x_1 (\theta)');
legend('Original Poles: P1', 'Faster Poles: P3');
grid on;
saveas(gcf, 'performance_comparison_corrected.png');

% Plot 2: Control Effort Comparison
figure('Name', 'Control Effort Comparison (Corrected Scale)');
plot(t1, vecnorm(u1), 'b-', 'LineWidth', 1.5);
hold on;
plot(t3, vecnorm(u3), 'g:', 'LineWidth', 2);
title('Control Effort Comparison');
xlabel('Time (s)');
ylabel('Magnitude of Control Input ||u(t)||');
legend('Original Poles: P1', 'Faster Poles: P3');
grid on;
saveas(gcf, 'effort_comparison_corrected.png');