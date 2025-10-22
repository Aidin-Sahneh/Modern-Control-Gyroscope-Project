clear;
clc;
close all;

% --- System Matrices and Controller Design ---
A = [0 1 0 0; 0 0 0 100; 0 0 0 1; 0 -100 0 0];
B = [0 0; 20 0; 0 0; 0 20];
C = [1 0 0 0; 0 0 1 0];
P = [-1, -2, -3, -4];
K = place(A, B, P);

% --- Calculate the Pre-compensator Gain Nr ---
DC_gain_matrix = C * (-(A-B*K)\B);
Nr = inv(DC_gain_matrix);

% --- Simulation Setup ---
tspan = [0 15];
x0 = [0; 0; 0; 0]; % Start from rest at the origin

% --- Define the Desired Reference Position ---
% Target: theta (x1) = 0.5 rad, phi (x3) = 0.2 rad
r = [0.5; 0.2];

% --- Run the Tracking Simulation ---
[t, x] = ode45(@(t,x) tracking_nonlinear_system(t, x, K, Nr, r), tspan, x0);

% --- Plot the Rlwesults ---
figure('Name', 'Reference Tracking Response');
y = C*x'; % Calculate the output y
plot(t, y(1,:), 'b-', 'LineWidth', 1.5);
hold on;
plot(t, y(2,:), 'r-', 'LineWidth', 1.5);

% Plot the reference lines for comparison
plot(t, r(1)*ones(size(t)), 'b--', 'LineWidth', 1);
plot(t, r(2)*ones(size(t)), 'r--', 'LineWidth', 1);

title('Output Tracking with Static Pre-compensator');
xlabel('Time (s)');
ylabel('Angle (rad)');
legend('Output y_1 (\theta)', 'Output y_2 (\phi)', 'Reference r_1', 'Reference r_2');
grid on;
saveas(gcf, 'tracking_response.png');