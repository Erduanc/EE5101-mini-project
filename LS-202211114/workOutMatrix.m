function [ABarD]=workOutMatrix(poles)
    ABarDRef = [       0       1       0       0       0       0;
                    0       0       1       0       0       0;
                    -336    -146    -21     0       0       0;
                    0       0       0       0       1       0;
                    0       0       0       0       0       1;
                    0       0       0       -36.003456      -36.400384 -12.6];
    ABarD = [   0       1       0       0       0       0;
                0       0       1       0       0       0;
                0       0       0       0       0       0;
                0       0       0       0       1       0;
                0       0       0       0       0       1;
                0       0       0       0       0       0];
    ABarD(3, 1) = poles(3)*poles(4)*poles(5);
    ABarD(3, 2) = -(poles(3)*poles(4)+poles(3)*poles(5)+poles(4)*poles(5));
    ABarD(3, 3) = (poles(3)+poles(4)+poles(5));
    ABarD(6, 4) = poles(1)*poles(2)*poles(6);
    ABarD(6, 5) = -(poles(1)*poles(2)+poles(1)*poles(6)+poles(2)*poles(6));
    ABarD(6, 6) = (poles(1)+poles(2)+poles(6));
end