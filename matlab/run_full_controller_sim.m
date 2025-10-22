clear;
clc;
close all;

% --- System Matrices ---
A = [0 1 0 0; 0 0 0 100; 0 0 0 1; 0 -100 0 0];
B = [0 0; 20 0; 0 0; 0 20];
C = [1 0 0 0; 0 0 1 0];

% --- Controller Design using High-Penalty LQR ---
Q = diag([10, 1, 10, 1]); 
R = eye(2) * 500;  % KEY CHANGE: Massively penalize control effort
K = lqr(A, B, Q, R);

fprintf('The High-Penalty LQR gain matrix K is:\n');
disp(K);

% --- Observer Design (Gentle Poles) ---
P_observer = [-3, -4, -5, -6]; 
L = place(A', C', P_observer)';

% --- Simulation Setup ---
tspan = [0 40]; % KEY CHANGE: Longer time for the gentle controller
x0 = [0.2; 0; 0.2; 0]; 
x_hat0 = x0; % Perfect initial guess
xa0 = [x0; x_hat0];

% --- Run the Full Simulation ---
[t, xa] = ode45(@(t,xa) observer_and_plant_system(t, xa, K, L, A, B, C), tspan, xa0);

% --- Analyze and Plot Results ---
x = xa(:, 1:4);

% Plot: System Stabilization
figure('Name', 'System Stabilization with High-Penalty LQR');
plot(t, x, 'LineWidth', 1.5);
title('System States Stabilized via Observer-Based LQR Control');
xlabel('Time (s)');
ylabel('State Values');
legend('x_1 (\theta)', 'x_2 (d\theta/dt)', 'x_3 (\phi)', 'x_4 (d\phi/dt)');
grid on;
saveas(gcf, 'final_stabilization_LQR_gentle.png');