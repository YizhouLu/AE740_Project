function sys_d = generateElecModel_test(Temp, model, dt)
% alpha0 = 3.613;                             % yintercept of SOC-OCV approximation
alpha1 = 0.4631;                            % slope of SOC-OCV curve between 20% and 80% SOC
R0  = getParamESC('R0Param',Temp,model);    % Ohmic resistance
R1  = getParamESC('RParam',Temp,model);     % RC circuit resistance
tau = getParamESC('RCParam',Temp,model);    % RC time constant
C1 = tau/R1;                                % RC circuit capacitance
Q = getParamESC('QParam',Temp,model);       % Battery capacity

A = [0,         0,  0;
     0,-1/(R1*C1),  0;
     0,         0,  0];
B = [  -1/(Q*3600);
       1/C1;
          0];
C = [alpha1, -1, 1];
D = -R0;

sys_d = c2d(ss(A,B,C,D), dt);
end