[A, B, C, x0] = buildStateSpaceModel();
% get the set point r:
a = 9; b = 8; c = 8; d = 7;
r0 = [-0.5+(a-b)/20; 0.1+(b-c)/(a+d+10)];
r = -1/10*C*inv(A)*B*[-0.5+(a-b)/20; 0.1+(b-c)/(a+d+10)];
% implement integrator servo control:
% augmented system params: v is 3 by 1
w = [-1; 1];
% disp(C);
ATld = [A zeros(6,3);
        -C zeros(3,3)];
BTld = [B; zeros(3, 2)];
BwTld = BTld;
BrTld = [zeros(6, 3); 
         eye(3 ,3)];
CTld = [C zeros(3, 6+3)];
% disp(ATld);
% CTld = [C 0];
% use LQR to stablize the augmented system:
Q = [3 0 0 0 0 0 0 0 0;
     0 6 0 0 0 0 0 0 0;
     0 0 9 0 0 0 0 0 0;
     0 0 0 5 0 0 0 0 0;
     0 0 0 0 2 0 0 0 0;
     0 0 0 0 0 7 0 0 0;
     0 0 0 0 0 0 2 0 0;
     0 0 0 0 0 0 0 20 0;
     0 0 0 0 0 0 0 0 21];
R = eye(2, 2);
KTld5_1 = real(getLQRController(ATld, BTld, Q, R));

% design the full order observer:
ADual = A.';
BDual = C.';
% Check if the dual system is controllable:
WcDual = [BDual ADual*BDual ADual^2*BDual ADual^3*BDual ADual^4*BDual ADual^5*BDual];
disp(rank(WcDual)); % full rank
% Do pole placement with unity rank method + ackermann's formula to stablize the dual system:
poles = [-1.8-0.872j -1.8+0.872j -6 -7 -8 -9]; % choose stable poles
ABarD = [       0       1      0       0       0       0;
                -42     -13    0       0       0       0;
                0       0      0       1       0       0;
                0       0       0       0       1       0;
                0       0       0       0       0       1;
                0       0       -288.027648 -327.206528 -137.200384 -20.6];
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
K5_1 = KDualBar*T;
disp(eig(ADual-BDual*K5_1));

L5_1 = K5_1';
disp(L5_1);
disp(eig(A-L5_1*C)); % verified the result of estimator gain of the original system
% % Choose q such that ADual, BDualq is controllable
% q = [2; 10; 2];
% BDualq = BDual*q;
% WcDualBq = [BDualq ADual*BDualq ADual^2*BDualq ADual^3*BDualq ADual^4*BDualq ADual^5*BDualq];
% disp(rank(WcDualBq)); % full rank, the new system is stable
% I = eye(6,6);
% phiDA = (ADual-I*poles(1))*(ADual-I*poles(2))*(ADual-I*poles(3))*(ADual-I*poles(4))*(ADual-I*poles(5))*(ADual-I*poles(6));
% KDualq5_1 = [0 0 0 0 0 1]*inv(WcDualBq)*phiDA;
% KDual5_1 = q*KDualq5_1;
% L5_1 = KDual5_1.';
% disp(L5_1);



