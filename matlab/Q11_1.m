% Original System Matrices
A = [0 1 0 0; 0 0 0 100; 0 0 0 1; 0 -100 0 0];
B = [0 0; 20 0; 0 0; 0 20];
C = [1 0 0 0; 0 0 1 0];

% Get dimensions
[n, m] = size(B); % n=4 (states), m=2 (inputs)
[p, ~] = size(C); % p=2 (outputs)

% Create the augmented system matrices
Aa = [A, zeros(n, p); -C, zeros(p, p)];
Ba = [B; zeros(p, m)];

% (Continue from the script in Step 1)

% Desired poles for the augmented system
Pa = [-1, -2, -3, -4, -0.5, -0.6];

% Calculate the augmented gain matrix Ka
Ka = place(Aa, Ba, Pa);

% Split Ka into Kp and Ki for clarity (optional)
Kp = Ka(:, 1:n);   % First 4 columns
Ki = Ka(:, n+1:end); % Last 2 columns

fprintf('The augmented gain matrix Ka is:\n');
disp(Ka);