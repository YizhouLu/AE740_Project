function [X_next, y, ocv, u] = MPC_pack(X0, model, coe)

% This function executes the MPC algorithm to generate an optimal current
% profile for a fast charge event.

% MPC parameters
global Param

N = struct();
N.prediction = 10;
N.control    = 1;

penalty   = struct();
penalty.Q = diag([0, 100, 0, 0, 0]);  % output = [Vt, e, i, z, Tc]
penalty.R = 1e-4;                     % input  = delta_i

% Use the cell capacity at 25 deg C since it is relatively invariant over
% the temperature range
Q_cell = getParamESC('QParam', 25, model); 
Q_pack = Param.num_parallel_cell * Param.num_strings * Q_cell;    

% constraints for the MPC problem
limit = struct();
% max voltage limit at OCV(z = 100) and min voltage limit at OCV(z = 0)
% since ideally we will never hit these limits and they will guarantee a
% constant voltage portion during charging assuming SOC setpoint is around
% 90%
Vt_max = 4.2025*Param.num_series; 
Vt_min = 2.8685*Param.num_series;
i_max  = 0;
i_min  = -Q_pack;

limit.y.max  = [Vt_max;  100; i_max;  0.9; 50];
limit.y.min  = [Vt_min; -100; i_min; -100;  0];
limit.du.max =  100;
limit.du.min = -100;

% initialize error for MPC formulation
X0(7) = X0(3) - Param.SOC_setpoint;

% define simulation parameters
t  = 0;  % initial time
dt = 1;  % sampling time

% MPC execution
[A_elec, B_elec, C_elec, D_elec] = generateElecPackModel(X0(10), X0(3), model, dt, coe);
[A_thml, B_thml, C_thml, D_thml] = generateThmlPackModel(X0(10), X0(6), model, dt);

A_aug = blkdiag(A_elec, A_thml);
B_aug = [B_elec;B_thml];
C_aug = blkdiag(C_elec, C_thml);
D_aug = [D_elec;D_thml];
sys_d = ss(A_aug, B_aug, C_aug, D_aug);

[H, L, G, W, T, ~, ~] = formQPMatrices(sys_d, N, penalty, limit);

Wtilde = W + T*X0;
q = L*X0;

options = optimset('Display','off');
dU = quadprog(H, q, G, Wtilde, [], [], [], [], [], options);

du = dU(1);     % delta u
u = du + X0(6); % actual control actuation

x_ode45_curr = [X0(3); X0(4); X0(10); X0(11)]; % [z, Vc, Tc, Ts]

[t_ode45, x_ode45] = ...
    ode45(@(t,x) Dynamics_pack(t, x, u, model), t + dt/10 : dt/10 : t + dt, x_ode45_curr);

x_ode45_next = x_ode45(end,:)';
t = t_ode45(end);


[y,ocv] = Output_pack(X0, u, model, coe); % [Vt, e, i, z, Tc]

X_next = zeros(size(X0));
X_next(1) = x_ode45_next(1) - X0(3);              % dz_k+1 = z_k+1 - z_k
X_next(2) = x_ode45_next(2) - X0(4);              % dVc_k+1 = Vc_k+1 - Vc_k
X_next(3) = x_ode45_next(1);                      % z_k+1
X_next(4) = x_ode45_next(2);                      % Vc_k+1
X_next(5) = 1;                                    % 1
X_next(6) = u;                                    % i_k
X_next(7) = x_ode45_next(1) - Param.SOC_setpoint; % z_k+1 - 0.8
X_next(8) = x_ode45_next(3) - X0(10);             % dTc_k+1 = Tc_k+1 - Tc_k
X_next(9) = x_ode45_next(4) - X0(11);             % dTs_k+1 = Ts_k+1 - Ts_k
X_next(10) = x_ode45_next(3);                     % Tc_k+1
X_next(11) = x_ode45_next(4);                     % Ts_k+1

end