function [ABarD]=workOutMatrix3(poles)
ABarDRef = [     0       1      0       0       0       0;
                -42     -13    0       0       0       0;
                0       0      0       1       0       0;
                0       0       0       0       1       0;
                0       0       0       0       0       1;
                0       0       -288.027648 -327.206528 -137.200384 -20.6];
    ABarD = [     0       1      0       0       0       0;
                -42     -13    0       0       0       0;
                0       0      0       1       0       0;
                0       0       0       0       1       0;
                0       0       0       0       0       1;
                0       0       -288.027648 -327.206528 -137.200384 -20.6];
    ABarD(2, 1) = -poles(3)*poles(4);
    ABarD(2, 2) = (poles(3)+poles(4));
    ABarD(6, 3) = -poles(1)*poles(2)*poles(5)*poles(6);
    ABarD(6, 4) = poles(1)*poles(2)*poles(5)+poles(1)*poles(2)*poles(6)+poles(1)*poles(5)*poles(6)+poles(2)*poles(5)*poles(6);
    ABarD(6, 5) = -(poles(1)*poles(2)+poles(1)*poles(5)+poles(1)*poles(6)+poles(2)*poles(5)+poles(2)*poles(6)+poles(5)*poles(6));
    ABarD(6, 6) = (poles(1)+poles(2)+poles(5)+poles(6));
end