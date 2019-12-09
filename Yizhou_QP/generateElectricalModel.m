function [A, B, C, D] = generateElectricalModel(Temp, model)
dt = 0.001;      % sampling time
alpha0 = 3.613;  % yintercept of SOC-OCV approximation
alpha1 = 0.4631; % slope of SOC-OCV curve between 20% and 80% SOC
R0  = getParamESC('R0Param',Temp,model); % Ohmic resistance
R1  = getParamESC('RParam',Temp,model);  % RC circuit resistance
tau = getParamESC('RCParam',Temp,model); % RC time constant
C1 = tau/R1;                             % RC circuit capacitance
Q = getParamESC('QParam',Temp,model);    % Battery capacity
% state = [delta_z; delta_Vc; i_last; e; z; Vc; 1]
A = [   1,                0, 0, 0, 0, 0, 0;
        0, exp(-dt/(R1*C1)), 0, 0, 0, 0, 0;
        0,                0, 1, 0, 0, 0, 0;
        1,                0, 0, 1, 0, 0, 0;
        1,                0, 0, 0, 1, 0, 0;
        0,                1, 0, 0, 0, 1, 0;
        0,                0, 0, 0, 0, 0, 1];
% input = delta_i
B = [                   -dt/Q;
      R1*(1-exp(-dt/(R1*C1)));
                            1;
                            0;
                            0;
                            0;
                            0];
% output = [e, Vt, i, z]                  
C = [0, 0,   0, 1,      0,  0,      0;
     0, 0, -R0, 0, alpha1, -1, alpha0;
     0, 0,   1, 0,      0,  0,      0;
     0, 0,   0, 0,      1,  0,      0];
  
D = [0; -R0; 1; 0];
end