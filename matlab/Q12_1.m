% System Matrices from previous questions
A = [0 1 0 0; 0 0 0 100; 0 0 0 1; 0 -100 0 0];
C = [1 0 0 0; 0 0 1 0];

% Desired observer poles (Step 2)
P_observer = [-10, -11, -12, -13];

% Calculate the observer gain L using the duality principle
L_transpose = place(A', C', P_observer);
L = L_transpose';

fprintf('The calculated observer gain matrix L is:\n');
disp(L);