clear;
clc;
close all;

% 1. Define symbolic variables for states and inputs
syms x1 x2 x3 x4 Tz Ty real

% Define the state vector and input vector
x = [x1; x2; x3; x4];
u = [Tz; Ty];


f1 = x2;
f2 = (Tz + 5*x4) / (0.05 * cos(x3));
f3 = x4;
f4 = (Ty - 5*x2 * cos(x3)) / 0.05;

f = [f1; f2; f3; f4]; % Vector of state derivatives

% 3. Define the operating point (x_op, u_op)
% For Tz=0, Ty=0 inputs, and if the system settles to zero,
% then the equilibrium point is x_op = [0;0;0;0].
x_op = [0; 0; 0; 0];
Tz_op = 0;
Ty_op = 0;
u_op = [Tz_op; Ty_op];

% Substitute operating point values into symbolic variables
x1_op = x_op(1);
x2_op = x_op(2);
x3_op = x_op(3);
x4_op = x_op(4);

% 4. Compute the Jacobian matrix A = df/dx at the operating point
A_sym = jacobian(f, x);
fprintf('Symbolic A matrix:\n');
disp(A_sym);

% Substitute the operating point values into the symbolic A matrix
A = subs(A_sym, [x1, x2, x3, x4, Tz, Ty], [x1_op, x2_op, x3_op, x4_op, Tz_op, Ty_op]);
A = double(A); % Convert to double precision
fprintf('\nNumerical A matrix at operating point:\n');
disp(A);

% 5. Compute the Jacobian matrix B = df/du at the operating point
B_sym = jacobian(f, u);
fprintf('\nSymbolic B matrix:\n');
disp(B_sym);

% Substitute the operating point values into the symbolic B matrix
B = subs(B_sym, [x1, x2, x3, x4, Tz, Ty], [x1_op, x2_op, x3_op, x4_op, Tz_op, Ty_op]);
B = double(B); % Convert to double precision
fprintf('\nNumerical B matrix at operating point:\n');
disp(B);

% 6. Display the linearized state-space model
fprintf('\nLinearized State-Space Model around x_op = [%g %g %g %g] and u_op = [%g %g]:\n', x_op, u_op);
sys_linear = ss(A, B, eye(4), zeros(4,2)); % C = eye(4) for outputting all states, D = zeros(4,2)
disp(sys_linear);

% Check the eigenvalues of A for stability of the linearized system
fprintf('\nEigenvalues of the A matrix (for stability of the linearized system):\n');
disp(eig(A));

