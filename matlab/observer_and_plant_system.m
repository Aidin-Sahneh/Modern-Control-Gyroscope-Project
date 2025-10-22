	function d_xa = observer_and_plant_system(t, xa, K, L, A, B, C)
	% Deconstruct the 8x1 state vector
	x = xa(1:4);     % True state
	x_hat = xa(5:8); % Estimated state
	
	% Controller using ESTIMATED state
	u = -K * x_hat; % for the close-loop ** IMPORTANT
    % u = [0; 0];  %for the open-loop ** IMPORTANT
	Tz = u(1);
	Ty = u(2);
	
	% Real Non-linear System Dynamics
	Ir = 0.05; IRws = 5;
	d_x = zeros(4,1);
	d_x(1) = x(2);
	d_x(2) = (Tz + IRws * x(4)) / (Ir * cos(x(3)));
	d_x(3) = x(4);
	d_x(4) = (Ty - IRws * x(2) * cos(x(3))) / Ir;
	y = C * x; % The real measurement
	
	% Linear Observer Dynamics
	y_hat = C * x_hat;
	d_x_hat = A*x_hat + B*u + L*(y - y_hat);
	
	% Combine to form the derivative of the full 8x1 state
	d_xa = [d_x; d_x_hat];
	end