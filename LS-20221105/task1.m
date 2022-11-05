[A, B, C, x0] = buildStateSpaceModel();
[ABar, BBar, T] = turnToControllableCanonicalForm(A, B);

% find damping ratio and natrual frequency:
% [dampingBound1, dampingBound2] = findDampingRatio(0.1);
% disp(dampingBound1); disp(dampingBound2);
zeta = 0.6; % choose damping ratio to be 0.6
[minFreq] = findNatrualFrequency(zeta, 5);
disp(minFreq); 
wn = 5; % choose natrual freq to be 5
% poles = [-1-5j*(1-0.2^2)^(1/2) -1+5j*(1-0.2^2)^(1/2) -3.5 -4 -4.5 -5]; % choose stable poles
% poles = [-1.2-1.6j -1.2+1.6j -4.8 -5.4 -5.76 -6.0]; % choose stable poles
poles = [-1.6-1.2j -1.6+1.2j -7.04 -7.36 -7.68 -8.0]; % choose stable poles
disp(poles);
% find the system controllability matrix:
Wc = [B A*B A^2*B A^3*B A^4*B A^5*B A^6*B];
disp(rank(Wc));
% use unity rank method to place poles:
% set q to be [1 1] s.t. (A, Bq) is controllable:
q = [1; 1];
Bq = B*q;
% find the new system controllability matrix:
WcNew = [Bq A*Bq A^2*Bq A^3*Bq A^4*Bq A^5*Bq];
disp(rank(WcNew));
% use Ackermann's theorem to design:
I = eye(6, 6);
phiDA = (A-I*poles(1))*(A-I*poles(2))*(A-I*poles(3))*(A-I*poles(4))*(A-I*poles(5))*(A-I*poles(6));
kNew = [0 0 0 0 0 1]*inv(WcNew)*phiDA;
disp(kNew);
K = q*kNew;
% prove to be a correct K:
disp(eig(A-B*K));
% poles:-1.0000 - 4.8990i  -1.0000 + 4.8990i  -3.5000 + 0.0000i  -4.0000 + 0.0000i  -4.5000 + 0.0000i  -5.0000 + 0.0000i
% ABar = ones(6, 6);
% BBar = [0 1;
%         1 0;
%         1 1;
%         1 1;
%         0 1;
%         1 0];
% poles = [-1-5j*(1-0.2^2)^(1/2) -1+5j*(1-0.2^2)^(1/2) -3.5 -4 -4.5 -5];
% disp(poles);
% % ABarD = [0          1           0           0           0           0;
% %          -25.00201 -2           0           0           0           0;
% %          0          0           0           1           0           0;
% %          0          0           0           0           1           0;
% %          0          0           0           0           0           1;
% %          0          0           -315        -301.75     -107.75     -17];
% ABarD = [0          1           0           0           0           0;
%          -25.00201 -2           0           0           0           0;
%          0          0           0           1           0           0;
%          0          0           0           0           1           0;
%          0          0           0           0           0           1;
%          0          0           -315        -301.75     -107.75     -17];
% disp(ABarD);
% disp(ABar); disp(BBar);
% KBar = BBar\(ABar-ABarD);
% disp(KBar);
% % proove K to be right:
% eigs = eig(ABar-BBar*KBar);
% disp(eigs);
% K = KBar*T;
% disp(K);
% % proove K to be right:
% disp(eig(A-B*K));