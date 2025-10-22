function d_xa = integral_control_system(t, xa, Ka, r)
    % Get dimensions
    n = 4; % Number of original states
    p = 2; % Number of integrator states
    
    % Deconstruct the augmented state vector
    x = xa(1:n);
    xi = xa(n+1:end);
    
    % Integral control law
    u = -Ka * xa;
    Tz = u(1);
    Ty = u(2);
    
    % Non-linear system dynamics
    Ir = 0.05;
    IRws = 5;
    d_x = zeros(n,1);
    d_x(1) = x(2);
    d_x(2) = (Tz + IRws * x(4)) / (Ir * cos(x(3)));
    d_x(3) = x(4);
    d_x(4) = (Ty - IRws * x(2) * cos(x(3))) / Ir;
    
    % Integrator dynamics (error calculation)
    y = [x(1); x(3)]; % Output y = [x1; x3]
    d_xi = r - y;
    
    % Combine to form the derivative of the augmented state
    d_xa = [d_x; d_xi];
end