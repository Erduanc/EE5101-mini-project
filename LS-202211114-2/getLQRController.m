function [K] = getLQRController(A, B, Q, R)
    nBym = size(B);
    n = nBym(1);
    m = nBym(2);
    % Solve ARE to get P
    gammaMat = [A  -B*inv(R)*B.';
                -Q -A.'         ];
    disp(gammaMat);
    [X, D] = eig(gammaMat);
%     disp(D);
%     disp(X);
    % Extract stable eigenvectors:
    stableX = [];
    ii = 1;
    while(ii <= 2*n)
        if(D(ii, ii) < 0)
            stableX = [stableX, X(1:end,ii)];
        end
        ii = ii+1;
    end
    disp(stableX)
    V = stableX(1:n, 1:end);
    MU = stableX(n+1:2*n, 1:end);
    P = MU*inv(V);
    % disp(V);
    % disp(MU);
    % Get K = R^-1*B'*P
    K = inv(R)*B.'*P;
end
