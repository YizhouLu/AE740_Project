function [y, OCV] = Output_pack(x, u, model, coe)
z  = x(3);
Vc = x(4);
e  = x(7);
Tc = x(10);
  
OCV = polyval(coe, z);
R0  = getParamESC('R0Param',Tc,model); % Ohmic resistance

num_series        = 96;
num_parallel_cell = 20;
num_strings       = 14;

% Series Resistance
Rtab      = 0.000125;    % 125 microOhm resistance for each tab
R0_eq     = R0 + 2*Rtab; % add tab resistance to cell resistance
R0_brick  = R0_eq./num_parallel_cell;
R0_string = R0_brick.*num_series;
R0_pack   = R0_string./num_strings;

Vt = OCV - R0_pack*u - Vc;

y = [Vt;e;u;z;Tc];
end