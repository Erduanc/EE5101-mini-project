A = [-1 1 0;
     0 1  1;
     0 0 2];
B = [1 1;
     0 0;
     0 1];
C = [1 3 -1;
     0 1 0];
x0 = [0, 0, 0, 0, 0, 0];

% A = [0 0 4 1;
%      10 13 2 8;
%      -3 -3 0 -2;
%      -10 -14 -5 -9];
% B = [-2 0;
%      4 -3;
%      -1 1;
%      -3 3];
% C = [1 3 -1;
%      0 1 0];
% x0 = [0, 0, 0, 0, 0, 0];

objectivePoles = [-1, -2, -3];

[Abar, Bbar, T] = turnToControllableCanonicalForm(A, B);
disp(Abar); disp(Bbar); disp(T);

ABar = [-1 7 0;
        0 0 1;
        0 -2 3];
BBar = [1 0;
        0 0;
        0 1];
ABarD = [-1 0 0;
          0 0 1;
          0 -6 -5];
KBar = BBar\(ABar-ABarD);
disp(K);
