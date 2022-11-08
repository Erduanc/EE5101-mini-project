% define the function to build the state space model of the system:
function [A, B, C, x0] = buildStateSpaceModel()
    % define the parameters:
    a = 9; b = 8; c = 8; d = 7;
    M_f = 2.14 + c/20;
    H_f = 0.18;
    M_r = 5.91 - b/10;
    H_r = 0.161;
    M_c = 1.74;
    H_c = 0.098;
    L_Ff = 0.05;
    L_F = 0.133;
    L_r = 0.128;
    L_R = 0.308+(a-d)/100;
    L_c = 0.259;
    J_x = 0.5+(c-d)/100;
    mu_x = 3.33 - b/20 + a*c/60;
    alpha = 15.5-a/3+b/2;
    beta = 27.5 - d/2;
    gamma = 11.5 + (a-c)/(b+d+3);
    delta = 60 + (a-b)*c/10;
    g = 9.8;
    % define the system:
    den = M_f*(H_f^2) + M_r*(H_r^2) + M_c*(H_c^2) + J_x;
    a51 = -M_c*g/den;
    a52 = (M_f*H_f + M_r*H_r + M_c*H_c)*g/den;
    a53 = (M_r*L_r*L_F + M_c*L_c*L_F + M_f*L_Ff*L_R)*g/((L_R + L_F)*den);
    a54 = -M_c*H_c*alpha/den;
    a55 = -mu_x/den;
    a56 = M_f*H_f*L_Ff*gamma/den;
    b51 = M_c*H_c*beta/den;
    b52 = -M_f*H_f*L_Ff*delta/den;
    
    A = [0 0 0 1 0 0; 
         0 0 0 0 1 0; 
         0 0 0 0 0 1; 
         0 6.5 -10 -alpha 0 0; 
         a51 a52 a53 a54 a55 a56;
         5 -3.6 0 0 0 -gamma];
    B = [0 0;
         0 0;
         0 0;
         beta 11.2;
         b51 b52;
         40 delta];
    C = [1 0 0 0 0 0;
         0 1 0 0 0 0;
         0 0 1 0 0 0];
    x0 = [0.2; -0.1; 0.15; -1; 0.8; 0];
end
