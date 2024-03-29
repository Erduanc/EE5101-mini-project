% Get the system matrices
[A, B, C, x0] = buildStateSpaceModel();
n = 6;
m = 2;
% Set n by n semi-positive diagonal Q and m by m positive diagonal R
% Q = eye(n, n);
% R = eye(m, m);
Q = [3  0   0   0   0   0;
     0  6   0   0   0   0;
     0  0   10   0   0   0;
     0  0   0   5   0   0;
     0  0   0   0   2   0;
     0  0   0   0   0   7];
R = eye(m, m);
K2_1 = real(getLQRController(A, B, Q, R));
K2_2 = real(getLQRController(A, B, 10*Q, R));
K2_3 = real(getLQRController(A, B, Q, 5*R));
K2_4 = real(getLQRController(A, B, 2*Q, 2*R));
% % Solve ARE to get P
% gammaMat = [A  -B*inv(R)*B.';
%             -Q -A.'         ];
% [X, D] = eig(gammaMat);
% % disp(D);
% % disp(X);
% % Extract stable eigenvectors:
% stableX = [];
% ii = 1;
% while(ii <= 2*n)
%     if(D(ii, ii) < 0)
%         stableX = [stableX, X(1:end,ii)];
%     end
%     ii = ii+1;
% end
% % disp(stableX);
% V = stableX(1:n, 1:end);
% MU = stableX(n+1:2*n, 1:end);
% P = MU*inv(V);
% % disp(V);
% % disp(MU);
% % Get K = R^-1*B'*P
% K = inv(R)*B.'*P;