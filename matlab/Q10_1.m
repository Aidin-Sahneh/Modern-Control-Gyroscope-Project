	% System Matrices and Controller from Q8
	A = [0 1 0 0; 0 0 0 100; 0 0 0 1; 0 -100 0 0];
	B = [0 0; 20 0; 0 0; 0 20];
	C = [1 0 0 0; 0 0 1 0];
	P = [-1, -2, -3, -4];
	K = place(A, B, P);
	
	% Calculate the DC gain matrix of the closed-loop system
	DC_gain_matrix = C * (-(A-B*K)\B);
	
	% Calculate its determinant to check for invertibility
	determinant_of_gain = det(DC_gain_matrix);
	
	fprintf('The DC gain matrix is:\n');
	disp(DC_gain_matrix);
	fprintf('\nThe determinant of the DC gain matrix is: %e\n', ...
	determinant_of_gain);

DC_gain_matrix = [6.6635, -0.1320; -0.1010, 2.5032];
Nr = inv(DC_gain_matrix);
disp('The pre-compensator matrix Nr is:');
disp(Nr);