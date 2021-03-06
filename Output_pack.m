function [y, OCV] = Output_pack(x, u, model, coe)

    global Param

    z  = x(3);
    Vc = x(4);
    e  = x(7);
    Tc = x(10);

    OCV = polyval(coe, z);
    R0  = getParamESC('R0Param',Tc,model); % Ohmic resistance

    % Series Resistance
    Rtab      = 0.000125;    % 125 microOhm resistance for each tab
    R0_eq     = R0 + 2*Rtab; % add tab resistance to cell resistance
    R0_brick  = R0_eq./Param.num_parallel_cell;
    R0_string = R0_brick.*Param.num_series;
    R0_pack   = R0_string./Param.num_strings;

    Vt = OCV - R0_pack*u - Vc;

    y = [Vt; e; u; z; Tc];
    
end