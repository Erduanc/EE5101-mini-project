[A, B, C, x0] = buildStateSpaceModel();
disp(C);
K = K2_1;
% Checking observability:
M = [C C*A C*A^2 C*A^3 C*A^4 C*A^5];
disp(rank(M)); % rank = 3, M is full col rank, observable
ADual = A';
BDual = C';
% place poles of the dual system to get KDual:
% use unity rank method + Ackerman's formula to solve:
% q = [1; 1; 1]; % design q s.t. A, Bq is controllable
% BqDual = BDual*q;
% WDual = [BqDual ADual*BqDual ADual^2*BqDual ADual^3*BqDual ADual^4*BqDual ADual^5*BqDual];
% disp(rank(WDual)); % rank = 6, WDual is full row rank, controllable
% % set poles to be:
% poles = [-1.6-1.2j -1.6+1.2j -7.04 -7.36 -7.68 -8.0];
% I = eye(6,6);
% phiDADual = (ADual-I*poles(1))*(ADual-I*poles(2))*(ADual-I*poles(3))*(ADual-I*poles(4))*(ADual-I*poles(5))*(ADual-I*poles(6));
% kDual = [0 0 0 0 0 1]*inv(WDual)*phiDADual;
% disp(eig(ADual-BqDual*kDual)); % verified the result of ackermann
% KDual = q*kDual;
% disp(eig(ADual-BDual*KDual)); % verified the result of pole palcement of the dual system
poles1 = [-1.8-0.872j -1.8+0.872j -6 -7 -8 -9]; % choose stable poles
poles2 = [-1.6-1.2j -1.6+1.2j -7.04 -7.36 -7.68 -8.0]; % choose stable poles
poles4 = [-1-5j*(1-0.2^2)^(1/2) -1+5j*(1-0.2^2)^(1/2) -3.5 -4 -4.5 -5]; % choose stable poles
poles3 = [-1.2-1.6j -1.2+1.6j -4.8 -5.4 -5.76 -6.0]; % choose stable poles

% ABarD = [       0       1      0       0       0       0;
%                 -42     -13    0       0       0       0;
%                 0       0      -8       0       0       0;
%                 0       0       0       0       1       0;
%                 0       0       0       0       0       1;
%                 0       0       0       -36.003456      -36.400384 -12.6];
ABarD = [       0       1      0       0       0       0;
                -42     -13    0       0       0       0;
                0       0      0       1       0       0;
                0       0       0       0       1       0;
                0       0       0       0       0       1;
                0       0       -288.027648 -327.206528 -137.200384 -20.6];

% ABarD = [       -8      0       0       0       0       0;
%                 0       0       1       0       0       0;
%                 0       -42     -13     0       0       0;
%                 0       0       0       0       1       0;
%                 0       0       0       0       0       1;
%                 0       0       0       -36.003456      -36.400384 -12.6];
ABarD = workOutMatrix3(poles1);

Wc = [BDual ADual*BDual ADual^2*BDual ADual^3*BDual ADual^4*BDual ADual^5*BDual];
Cc = Wc(1:end,1:6);
Cc = [Cc(1:end, 1) Cc(1:end, 4) Cc(1:end, 2) Cc(1:end, 5) Cc(1:end, 3) Cc(1:end, 6)];
CcInv = inv(Cc);
d1 = 2; d2 = 2; d3 = 2;
q2 = CcInv(2, 1:end); q4 = CcInv(4, 1:end); q6 = CcInv(6, 1:end);
T = [q2;
     q2*ADual;
     q4;
     q4*ADual;
     q6;
     q6*ADual];
ADualBar = T*ADual*inv(T);
BDualBar = T*BDual;

KDualBar = BDualBar\(ADualBar-ABarD);
K3_1 = KDualBar*T;
disp(eig(ADual-BDual*K3_1));

L3_1 = K3_1';
disp(L3_1);
disp(eig(A-L3_1*C)); % verified the result of estimator gain of the original system

