function PackCurrent = BattPowerToCurrent(BattPower,X,OCV,model)
   
    global Param
    
    Tc = X(10);
    R0  = getParamESC('R0Param',Tc,model); % Ohmic resistance

    % Series Resistance
    Rtab      = 0.000125;    % 125 microOhm resistance for each tab
    R0_eq     = R0 + 2*Rtab; % add tab resistance to cell resistance
    R0_brick  = R0_eq./Param.num_parallel_cell;
    R0_string = R0_brick.*Param.num_series;
    R0_pack   = R0_string./Param.num_strings;
    
    Vc = X(4);
    
    Vfixed = OCV - Vc;
    
    PackCurrent = (Vfixed - sqrt((Vfixed^2) - 4 * R0_pack * BattPower)) / ...
        (2* R0_pack);
    
 
    
end

