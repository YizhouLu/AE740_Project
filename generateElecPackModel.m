function [A_aug, B_aug, C_aug, D_aug] = generateElecPackModel(Temp, SOC, model, dt, coe)

global Param

OCV =  polyval(coe,SOC);

R0  = getParamESC('R0Param',Temp,model); % Ohmic resistance
R1  = getParamESC('RParam',Temp,model);  % RC circuit resistance
tau = getParamESC('RCParam',Temp,model); % RC time constant
C1 = tau/R1;                             % RC circuit capacitance


Q_cell = getParamESC('QParam',Temp,model); % Battery capacity
Q_pack = Param.num_parallel_cell * Param.num_strings * Q_cell;    


% Series Resistance
Rtab      = 0.000125;    % 125 microOhm resistance for each tab
R0_eq     = R0 + 2*Rtab; % add tab resistance to cell resistance
R0_brick  = R0_eq./Param.num_parallel_cell;
R0_string = R0_brick.*Param.num_series;
R0_pack   = R0_string./Param.num_strings;


% The delta subsystem [delta_z, delta_Vc] due to linearization
A = [0,           0;
     0, -1/(R1*C1)];
 
B = [ -1/(Q_pack*3600);
                 1/(C1*Param.num_parallel_cell * Param.num_strings)];
             
C = zeros(1,2);

D = 0;

% The actual electrical subsystem [z, Vc, 1]
A2 = [0,          0,  0;
      0, -1/(R1*C1),  0;
      0,          0,  0];
  
B2 = [ -1/(Q_pack*3600);
                   1/(C1*Param.num_parallel_cell * Param.num_strings);
                     0];
C2 = [0, -1, OCV];

D2 = -R0_pack;

sys1_d = c2d(ss(A,B,C,D), dt);
sys2_d = c2d(ss(A2,B2,C2,D2), dt);

% state = [delta_z; delta_Vc; z; Vc; 1; i_last; e]
A_aug          = zeros(7);
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