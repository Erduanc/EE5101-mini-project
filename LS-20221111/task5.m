[A, B, C, x0] = buildStateSpaceModel();
% get the set point r:
a = 9; b = 8; c = 8; d = 7;
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
Q = [1 0 0 0 0 0 0 0 0;
     0 1 0 0 0 0 0 0 0;
     0 0 1 0 0 0 0 0 0;
     0 0 0 1 0 0 0 0 0;
     0 0 0 0 1 0 0 0 0;
     0 0 0 0 0 1 0 0 0;
     0 0 0 0 0 0 2 0 0;
     0 0 0 0 0 0 0 10 0;
     0 0 0 0 0 0 0 0 5];
R = eye(2, 2);
KTld5_1 = real(getLQRController(ATld, BTld, Q, R));
% design the full order observer:
ADual = A.';
BDual = C.';
% Check if the dual system is controllable:
WcDual = [BDual ADual*BDual ADual^2*BDual ADual^3*BDual ADual^4*BDual ADual^5*BDual];
disp(rank(WcDual)); % full rank
% Do pole placement with unity rank method + ackermann's formula to stablize the dual system:
poles = [-1.6-1.2j -1.6+1.2j -7.04 -7.36 -7.68 -8.0];
% Choose q such that ADual, BDualq is controllable
q = [1; 1; 1];
BDualq = BDual*q;
WcDualBq = [BDualq ADual*BDualq ADual^2*BDualq ADual^3*BDualq ADual^4*BDualq ADual^5*BDualq];
disp(rank(WcDualBq)); % full rank, the new system is stable
I = eye(6,6);
phiDA = (ADual-I*poles(1))*(ADual-I*poles(2))*(ADual-I*poles(3))*(ADual-I*poles(4))*(ADual-I*poles(5))*(ADual-I*poles(6));
KDualq5_1 = [0 0 0 0 0 1]*inv(WcDualBq)*phiDA;
KDual5_1 = q*KDualq5_1;
L5_1 = KDual5_1.';
disp(L5_1);
% K5_1 = KTld5_1(1:end, 1:6);
% disp(K5_1);
% gamma = [ATld -BTld*inv(R)*BTld.'
%         -Q      -ATld.'];
% [X, D] = eig(gamma);
% disp(D);
% stableX = [];
% ii = 1;
% while(ii <= 2*9)
%     if(D(ii, ii) < 0)
%         stableX = [stableX, X(1:end, ii)];
%     end
%     ii = ii+1;
% end
% V = stableX(1:9, 1:end);
% Mu = stableX(10:end, 1:end);
% disp(V);
% disp(Mu);



