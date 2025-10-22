	A = [0 1 0 0;
	0 0 0 100;
	0 0 0 1;
	0 -100 0 0];
	B = [0 0;
	0 20; % Note: Corrected from user's Simulink model
	0 0;
	20 0];
	C = [1 0 0 0;
	0 0 1 0];
	
	% Method 1: Kalman Rank Test
	Co = ctrb(A, B);
	rank_Co = rank(Co);
	Ob = obsv(A, C);
	rank_Ob = rank(Ob);
	disp(['Rank of controllability matrix: ', num2str(rank_Co)]);
	disp(['Rank of observability matrix: ', num2str(rank_Ob)]);
	
	% Method 2: PBH Test
	lambda = eig(A);
	n = size(A,1);
	fprintf('\n--- PBH Test ---\n');
	for i = 1:length(lambda)
	PBH_ctrl = [lambda(i)*eye(n) - A, B];
	PBH_obsv = [lambda(i)*eye(n) - A; C];
	fprintf('For lambda = %.2f%+.2fi, rank_ctrl = %d, rank_obsv = %d\n', ...
	real(lambda(i)), imag(lambda(i)), rank(PBH_ctrl), rank(PBH_obsv));
	end
	
	% Method 3: Jordan Form Test
	[V, J] = jordan(A);
	disp('Jordan form of A:'); disp(J);
	B_jordan = inv(V) * B;
	disp('B in Jordan coordinates:'); disp(B_jordan);
	C_jordan = C * V;
	disp('C in Jordan coordinates:'); disp(C_jordan);