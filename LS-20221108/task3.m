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
q = [1; 1; 1]; % design q s.t. A, Bq is controllable
BqDual = BDual*q;
WDual = [BqDual ADual*BqDual ADual^2*BqDual ADual^3*BqDual ADual^4*BqDual ADual^5*BqDual];
disp(rank(WDual)); % rank = 6, WDual is full row rank, controllable
% set poles to be:
poles = [-1.6-1.2j -1.6+1.2j -7.04 -7.36 -7.68 -8.0];
phiDADual = (ADual-I*poles(1))*(ADual-I*poles(2))*(ADual-I*poles(3))*(ADual-I*poles(4))*(ADual-I*poles(5))*(ADual-I*poles(6));
kDual = [0 0 0 0 0 1]*inv(WDual)*phiDADual;
disp(eig(ADual-BqDual*kDual)); % verified the result of ackermann
KDual = q*kDual;
disp(eig(ADual-BDual*KDual)); % verified the result of pole palcement of the dual system
L3_1 = KDual';
disp(eig(A-L3_1*C)); % verified the result of estimator gain of the original system

