function [A_aug, B_aug, C_aug, D_aug] = generateElecModel(Temp, model, dt)
alpha0 = 3.613;  % yintercept of SOC-OCV approximation
alpha1 = 0.4631; % slope of SOC-OCV curve between 20% and 80% SOC
R0  = getParamESC('R0Param',Temp,model); % Ohmic resistance
R1  = getParamESC('RParam',Temp,model);  % RC circuit resistance
tau = getParamESC('RCParam',Temp,model); % RC time constant
C1 = tau/R1;                             % RC circuit capacitance
Q = getParamESC('QParam',Temp,model);    % Battery capacity

A = [0,         0;
      0,-1/(R1*C1)];
B = [ -1/(Q*3600);
       1/C1];
C = zeros(1,2);
D = 0;

A2 = [0,         0,  0;
      0,-1/(R1*C1),  0;
      0,         0,  0];
B2 = [ -1/(Q*3600);
       1/C1;
         0];
C2 = [alpha1, -1, alpha0];
D2 = -R0;

sys1_d = c2d(ss(A,B,C,D), dt);
sys2_d = c2d(ss(A2,B2,C2,D2), dt);
% state = [delta_z; delta_Vc; z; Vc; 1; i_last; e]
A_aug = zeros(7);
A_aug(1:2,1:2) = sys1_d.A;
A_aug(3:5,3:5) = sys2_d.A;  
A_aug(3:5,6)   = sys2_d.B;  
A_aug(6,:)     = [0, 0, 0, 0, 0, 1, 0];
A_aug(7,:)     = [1, 0, 0, 0, 0, 0, 1];
% input = delta_i
B_aug = [sys1_d.B;
         sys2_d.B;
                1;
                0];
% output = [Vt, e, i, z]                  
C_aug = [0, 0, sys2_d.C, sys2_d.D, 0; 
         0, 0,  0, 0, 0,        0, 1;
         0, 0,  0, 0, 0,        1, 0;
         0, 0,  1, 0, 0,        0, 0];
  
D_aug = [sys2_d.D;
                0;
                1;
                0];
end