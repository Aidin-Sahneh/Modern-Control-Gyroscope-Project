	clear; clc; close all;
	% --- Input Parameters (Open Loop) ---
	Tz_func = @(t) 0; % Tz is constant zero
	Ty_func = @(t) 0; % Ty is constant zero
	
	% --- Different Initial Conditions ---
	x0_case1 = [0; 0; 0; 0];
	x0_case2 = [0.1; 0.5; pi/4; 0.1];
	x0_case3 = [1; 0; pi/4; 0]; 
	
	% --- Simulation Time Span ---
	tspan = [0 20];
	
	%% --- Case 1: Zero Initial Conditions ---
	fprintf('Simulating Case 1...\n');
	[t1, x1] = ode45(@(t, x) nonlinear_system(t, x, Tz_func, Ty_func), tspan, x0_case1);
	
	fig1 = figure('Name', 'Case 1: Response from Origin');
	subplot(4,1,1);
	plot(t1, x1(:,1), 'LineWidth', 1.5);
	title('System Behavior for x_0 = [0, 0, 0, 0]');
	ylabel('x_1 (\theta)');
	grid on;
	
	subplot(4,1,2);
	plot(t1, x1(:,2), 'LineWidth', 1.5);
	ylabel('x_2 (d\theta/dt)');
	grid on;
	
	subplot(4,1,3);
	plot(t1, x1(:,3), 'LineWidth', 1.5);
	ylabel('x_3 (\phi)');
	grid on;
	
	subplot(4,1,4);
	plot(t1, x1(:,4), 'LineWidth', 1.5);
	xlabel('Time (s)');
	ylabel('x_4 (d\phi/dt)');
	grid on;
	saveas(fig1, 'case1.png'); % Save the figure
	
	%% --- Case 2: Non-Zero Initial Conditions ---
	fprintf('Simulating Case 2...\n');
	[t2, x2] = ode45(@(t, x) nonlinear_system(t, x, Tz_func, Ty_func), tspan, x0_case2);
	
	fig2 = figure('Name', 'Case 2: Response from Non-Zero State');
	subplot(4,1,1);
	plot(t2, x2(:,1), 'LineWidth', 1.5);
	title('System Behavior for x_0 = [0.1, 0.5, \pi/4, 0.1]');
	ylabel('x_1 (\theta)');
	grid on;
	
	subplot(4,1,2);
	plot(t2, x2(:,2), 'LineWidth', 1.5);
	ylabel('x_2 (d\theta/dt)');
	grid on;
	
	subplot(4,1,3);
	plot(t2, x2(:,3), 'LineWidth', 1.5);
	ylabel('x_3 (\phi)');
	grid on;
	
	subplot(4,1,4);
	plot(t2, x2(:,4), 'LineWidth', 1.5);
	xlabel('Time (s)');
	ylabel('x_4 (d\phi/dt)');
	grid on;
	saveas(fig2, 'case2.png'); % Save the figure
	
	%% --- Case 3: Non-Zero Position, Zero Velocity ---
	fprintf('Simulating Case 3...\n');
	[t3, x3] = ode45(@(t, x) nonlinear_system(t, x, Tz_func, Ty_func), tspan, x0_case3);
	
	fig3 = figure('Name', 'Case 3: Response from Non-Zero Position');
	subplot(4,1,1);
	plot(t3, x3(:,1), 'LineWidth', 1.5);
	title('System Behavior for x_0 = [1, 0, \pi/4, 0]');
	ylabel('x_1 (\theta)');
	grid on;
	
	subplot(4,1,2);
	plot(t3, x3(:,2), 'LineWidth', 1.5);
	ylabel('x_2 (d\theta/dt)');
	grid on;
	
	subplot(4,1,3);
	plot(t3, x3(:,3), 'LineWidth', 1.5);
	ylabel('x_3 (\phi)');
	grid on;
	
	subplot(4,1,4);
	plot(t3, x3(:,4), 'LineWidth', 1.5);
	xlabel('Time (s)');
	ylabel('x_4 (d\phi/dt)');
	grid on;
	saveas(fig3, 'case3.png'); % Save the figure
	
	fprintf('Simulations complete. Figures saved as case1.png, case2.png, case3.png.\n');