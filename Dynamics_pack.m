function x_dot = Dynamics_pack(~, x, u, model)

    global Param
    
    Vc = x(2);
    Tc = x(3);
    Ts = x(4);

    R1  = getParamESC('RParam',Tc,model);  % RC circuit resistance
    tau = getParamESC('RCParam',Tc,model); % RC time constant
    C1 = tau/R1;                           % RC circuit capacitance

    Q_cell = getParamESC('QParam',Tc,model); % Battery capacity
    Q_pack = Param.num_parallel_cell * Param.num_strings * Q_cell;    

    Tamb = 25;

    u_cell = u /(Param.num_parallel_cell * Param.num_strings);

    z_dot  = -1/(Q_pack*3600) * u;
    Vc_dot = -1/(R1*C1) * Vc + 1/C1 * u_cell;

    Tc_dot = Param.Re/Param.Cc * u_cell^2 - (Tc - Ts)/(Param.Cc * Param.Rc);
    Ts_dot = (Tc - Ts)/(Param.Cs * Param.Rc) - (Ts - Tamb)/(Param.Cs * Param.Ru);

    x_dot = [z_dot; Vc_dot; Tc_dot; Ts_dot];

end