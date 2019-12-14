function x_dot = Dynamics_pack(~, x, u, model)

Vc = x(2);
Tc = x(3);
Ts = x(4);

R1  = getParamESC('RParam',Tc,model);  % RC circuit resistance
tau = getParamESC('RCParam',Tc,model); % RC time constant
C1 = tau/R1;                           % RC circuit capacitance

num_series        = 96;
num_parallel_cell = 20;
num_strings       = 14;

Q_cell = getParamESC('QParam',Tc,model); % Battery capacity
Q_pack = num_parallel_cell * num_strings * Q_cell;    

Cc = 80;   % J/K  (tune)
Cs = 10;   % J/K  (tune)
Re = 1;    % Ohms (combo of R1 and R0)
Rc = 1;    % K/W  (tune)
Ru = 4;    % K/W  (tune)

Tamb = 25;

u_cell = u /(num_parallel_cell * num_strings);

z_dot  = -1/(Q_pack*3600) * u;
Vc_dot = -1/(R1*C1) * Vc + 1/C1 * u_cell;

Tc_dot = Re/Cc * u_cell^2 - (Tc - Ts)/(Cc * Rc);
Ts_dot = (Tc - Ts)/(Cs * Rc) - (Ts - Tamb)/(Cs * Ru);

x_dot = [z_dot;Vc_dot;Tc_dot;Ts_dot];

end