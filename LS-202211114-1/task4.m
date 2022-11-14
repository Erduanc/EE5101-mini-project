[A, B, C, x0] = buildStateSpaceModel();
C4 = [1 0 0 0 0 0;
      0 0 1 0 0 0];
% work out sigmas, there are m = 2 sigmas:
% for sigma1:
jj = 1; c1T = C4(1, 1:6);
term = c1T*A^(jj-1)*B;
disp(term);
jj = 2;
term = c1T*A^(jj-1)*B;
disp(term);
sigma1 = jj; % term ~= [0 0];
% for sigma2:
jj = 1; c2T = C4(2, 1:6);
term = c2T*A^(jj-1)*B;
disp(term);
jj = 2;
term = c2T*A^(jj-1)*B;
disp(term);
sigma2 = jj;

% decide poles for each output, there are 2 output terminals:
poles1 = [-1.8-0.872j -1.8+0.872j]; % sigma1 poles
poles2 = [-9 -8]; % sigma2 poles
I = eye(6,6);
phiDA1 = (A-poles1(1)*I)*(A-poles1(2)*I);
phiDA2 = (A-poles2(1)*I)*(A-poles2(2)*I);
% workout B* and C**:
BStar = [c1T*A^(sigma1-1)*B;
         c2T*A^(sigma2-1)*B];
CStar = [c1T*A^(sigma1);
         c2T*A^(sigma2)];
CStarStar = [c1T*phiDA1;
             c2T*phiDA2];
% workout F and K:
F4_1 = inv(BStar);
K4_1 = inv(BStar)*CStar;
K4_2 = inv(BStar)*CStarStar;