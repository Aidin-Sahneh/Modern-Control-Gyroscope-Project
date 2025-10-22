function dxdt = closed_loop_nonlinear_system(t, x, K)
    % Calculate the state feedback control input
    u = -K * x;
    Tz = u(1);
    Ty = u(2);
    
    % Original non-linear system parameters
    Ir = 0.05;
    IRws = 5;
    
    % Non-linear state equations
    dxdt = zeros(4,1);
    dxdt(1) = x(2);
    dxdt(2) = (Tz + IRws * x(4)) / (Ir * cos(x(3)));
    dxdt(3) = x(4);
    dxdt(4) = (Ty - IRws * x(2) * cos(x(3))) / Ir;
end