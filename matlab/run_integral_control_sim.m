clear;
clc;
close all;

% --- Controller Design (from previous steps) ---
A = [0 1 0 0; 0 0 0 100; 0 0 0 1; 0 -100 0 0];
B = [0 0; 20 0; 0 0; 0 20];
C = [1 0 0 0; 0 0 1 0];
[n, m] = size(B);
[p, ~] = size(C);
Aa = [A, zeros(n, p); -C, zeros(p, p)];
Ba = [B; zeros(p, m)];
Pa = [-1, -2, -3, -4, -0.5, -0.6];
Ka = place(Aa, Ba, Pa);

% --- Simulation Setup ---
tspan = [0 15];
x0 = [0; 0; 0; 0]; % Initial conditions for original system
xi0 = [0; 0]; % Initial conditions for integrator states
xa0 = [x0; xi0]; % Full augmented state vector

% --- Define the Desired Reference Position ---
r = [0.5; 0.2]; % Target: theta=0.5 rad, phi=0.2 rad

% --- Run the Integral Control Simulation ---
[t, xa] = ode45(@(t,xa) integral_control_system(t, xa, Ka, r), tspan, xa0);

% --- Plot the Results ---
figure('Name', 'Reference Tracking with Integral Control');
x = xa(:, 1:n); % Extract original states
y = C*x'; % Calculate the output y
plot(t, y(1,:), 'b-', 'LineWidth', 1.5);
hold on;
plot(t, y(2,:), 'r-', 'LineWidth', 1.5);

% Plot the reference lines
plot(t, r(1)*ones(size(t)), 'b--', 'LineWidth', 1);
plot(t, r(2)*ones(size(t)), 'r--', 'LineWidth', 1);

title('Output Tracking with Integral Control');
xlabel('Time (s)');
ylabel('Angle (rad)');
legend('Output y_1 (\theta)', 'Output y_2 (\phi)', 'Reference r_1', 'Reference r_2');
grid on;
saveas(gcf, 'integral_control_response.png');