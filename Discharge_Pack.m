function [x_ode45_next, y, ocv] = Discharge_Pack(Bus1, model, coe, Dchg_i)

    u = Dchg_i;

    t  = 0;  % initial time
    dt = 1;  % sampling time

    x_ode45_curr = [Bus1.X(3); Bus1.X(4); Bus1.X(10); Bus1.X(11)]; % [z, Vc, Tc, Ts]

    [~, x_ode45] = ...
        ode45(@(t,x) Dynamics_pack(t, x, u, model), t + dt/10 : dt/10 : t + dt, x_ode45_curr);

    x_ode45_next = x_ode45(end,:)';
    
    [y,ocv] = Output_pack(Bus1.X, u, model, coe); % [Vt, e, i, z, Tc]

end