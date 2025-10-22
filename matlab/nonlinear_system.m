function dxdt = nonlinear_system(t, x, Tz_func, Ty_func)
    % State variables: x(1)=theta, x(2)=theta_dot, 
    % x(3)=phi, x(4)=phi_dot
    
    % System Parameters
    Ir = 0.05;
    IRws = 5; % IR * ws = 0.01 * 500
    
    % Get input torques at the current time t
    Tz = Tz_func(t);
    Ty = Ty_func(t);
    
    % Initialize the state derivative vector
    dxdt = zeros(4,1);
    
    % State-space equations from Question 1
    dxdt(1) = x(2);
    dxdt(2) = (Tz + IRws * x(4)) ...
                / (Ir * cos(x(3)));
    dxdt(3) = x(4);
    dxdt(4) = (Ty - IRws * x(2) * cos(x(3))) / Ir;
end