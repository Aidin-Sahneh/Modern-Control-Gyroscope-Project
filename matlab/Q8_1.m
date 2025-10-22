% System Matrices from Question 5
A = [0 1 0 0;
     0 0 0 100;
     0 0 0 1;
     0 -100 0 0];
B = [0 0;
     20 0;
     0 0;
     0 20];

% Desired pole locations (Step 2)
P = [-1, -2, -3, -4];

% Calculate the state feedback gain matrix K
K = place(A, B, P);

disp('The calculated gain matrix K is:');
disp(K);