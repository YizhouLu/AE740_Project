clc; clear; close all

load('SAMmodel.mat')
load('SOC_OCV_pack_coe.mat')

Temp = 25; % C
%% MPC parameters
N = struct();
N.prediction = 10;
N.control = 1;

penalty = struct();
penalty.Q = diag([0, 100, 0, 0, 0]); % output = [Vt, e, i, z, Tc]
penalty.R = 1e-4;                     % input = di

%% Pack Configuration to match Proterra DuoPower Drivetrain
num_series        = 96;
num_parallel_cell = 20;
num_strings       = 14;
 
% % Scale the open circuit voltage
% OCV0 = model.OCV0;
% OCVrel = model.OCVrel;
% 
% OCV = OCV0+ 25*OCVrel;
% SOC = model.SOC;
% 
% OCV0_pack   = num_series.*OCV0;
% OCVrel_pack = num_series.*OCVrel;

Q_cell = getParamESC('QParam',Temp,model);
Q_pack = num_parallel_cell * num_strings * Q_cell;

limit = struct();
% Vt_max = GetOCV(0.8, Temp, model);
Vt_max = 4.2*num_series; % GetOCV(0.8, Temp, model);
Vt_min = 3.0*num_series; % GetOCV(0.2, Temp, model);
i_min = -Q_pack;
limit.y.max = [Vt_max;  100;     0; 0.9; 50000];
limit.y.min = [Vt_min; -100; i_min; 0.2;  0];
limit.du.max =  100;
limit.du.min = -100;

lambda = ones(2 * N.control * 1 + 2 * N.prediction * 5,1);

%% Model initial conditions
%    [dz; dVc;   z; Vc; 1; i_last;    e; dTc; dTs; Tc; Ts]
X0 = [ 0;   0; 0.2;  0; 1;      0; -0.7;   0;   0; 25; 25];
%    di
dU0 = 0;
SOC_setpoint = 0.9;

%% Simulation parameters
t  = 0;         % initial time
dt = 1;         % sampling time
Nsim = 7200;    % sampling steps

X = zeros(11, Nsim+1); X(:,1) = X0;
U = zeros( 1, Nsim);
Y = zeros( 5, Nsim);
OCV = zeros(1, Nsim);

for i = 1:Nsim
    tic
    [A_elec, B_elec, C_elec, D_elec] = generateElecModel(X0(10), X0(3), model, dt, coe);
    [A_thml, B_thml, C_thml, D_thml] = generateThmlModel(X0(10), X0(6), model, dt);
    
    A_aug = blkdiag(A_elec, A_thml);
    B_aug = [B_elec;B_thml];
    C_aug = blkdiag(C_elec, C_thml);
    D_aug = [D_elec;D_thml];
    sys_d = ss(A_aug, B_aug, C_aug, D_aug);
    
    [H, L, G, W, T, Phi, Gamma] = formQPMatrices(sys_d, N, penalty, limit);
    Wtilde = W + T*X0;
    q = L*X0;
%     [dU, lambda] = myQP(H, q, G, Wtilde, lambda, 200);
    options = optimset('Display','off');
    [dU,~,~,~,lambda] = quadprog(H, q, G, Wtilde, [], [], [], [], dU0, options);
    
    du = dU(1);     % delta u
    u = du + X0(6); % actual control actuation
    U(1,i) = u;

    x_ode45_curr = [X0(3); X0(4); X0(10); X0(11)]; % z, Vc, Tc, Ts
    [t_ode45, x_ode45] = ode45(@(t,x) Dynamics(t, x, u, model), t + dt/10 : dt/10 : t + dt, x_ode45_curr);
    x_ode45_next = x_ode45(end,:)';
    t = t_ode45(end);
    
    
    [y,ocv] = Output(X0, u, model, coe);% [Vt, e, i, z, Tc]
    Y(:,i) = y;
    OCV(:,i) = ocv;
    
    X_next = zeros(size(X0));
    X_next(1) = x_ode45_next(1) - X0(3);    % dz_k+1 = z_k+1 - z_k
    X_next(2) = x_ode45_next(2) - X0(4);    % dVc_k+1 = Vc_k+1 - Vc_k
    X_next(3) = x_ode45_next(1);            % z_k+1
    X_next(4) = x_ode45_next(2);            % Vc_k+1
    X_next(5) = 1;                          % 1 
    X_next(6) = u;                          % i_k
    X_next(7) = x_ode45_next(1) - SOC_setpoint; % z_k+1 - 0.8
    X_next(8) = x_ode45_next(3) - X0(10);   % dTc_k+1 = Tc_k+1 - Tc_k
    X_next(9) = x_ode45_next(4) - X0(11);   % dTs_k+1 = Ts_k+1 - Ts_k
    X_next(10) = x_ode45_next(3);           % Tc_k+1
    X_next(11) = x_ode45_next(4);           % Ts_k+1
    
    X(:,i+1) = X_next;
    
    X0 = X_next;
    dU0 = dU;
    toc
end

%% Plot results
% close all
t = (1:1:Nsim)*dt;

% figure('Name','Fast Charge Output');
set(gcf,'Color','White','Units','Normalized','Position',[0.2 0.2 0.6 0.6]);

h_ax1(2) = subplot(231); % voltage
plot(t,Y(1,:),'b','LineWidth',2);
hold on; grid on
plot(t,OCV(1,:),'k--','LineWidth',2);
plot(t,limit.y.max(1)*ones(size(t)),'--r','LineWidth',2);
plot(t,limit.y.min(1)*ones(size(t)),'--r','LineWidth',2);
xlabel('Time [s]');
legend('terminal voltage','open source voltage', 'Location','best');
set(gca,'FontSize',14);

h_ax1(1) = subplot(232); % error
plot(t,Y(2,:),'b','LineWidth',2);
hold on; grid on
% plot(t,Y_linear(2,:),'k--','LineWidth',2);
% no constraints on error so not plotted
xlabel('Time [s]');
legend('error','Location','best');
set(gca,'FontSize',14);


h_ax1(1) = subplot(233); % current
plot(t,Y(3,:),'b','LineWidth',2);
hold on; grid on
% plot(t,Y_linear(3,:),'k--','LineWidth',2);
plot(t,limit.y.max(3)*ones(size(t)),'--r','LineWidth',2);
plot(t,limit.y.min(3)*ones(size(t)),'--r','LineWidth',2);
xlabel('Time [s]');
legend('current','Location','best');
set(gca,'FontSize',14);

h_ax1(1) = subplot(234); % SOC
plot(t,Y(4,:),'b','LineWidth',2);
hold on; grid on
% plot(t,Y_linear(4,:),'k--','LineWidth',2);
plot(t,limit.y.max(4)*ones(size(t)),'--r','LineWidth',2);
plot(t,limit.y.min(4)*ones(size(t)),'--r','LineWidth',2);
xlabel('Time [s]');
legend('State of Charge','Location','best');
set(gca,'FontSize',14);

h_ax1(1) = subplot(235); % Tc
plot(t,Y(5,:),'b','LineWidth',2);
hold on; grid on
% plot(t,Y_linear(5,:),'k--','LineWidth',2);
plot(t,limit.y.max(5)*ones(size(t)),'--r','LineWidth',2);
plot(t,limit.y.min(5)*ones(size(t)),'--r','LineWidth',2);
xlabel('Time [s]');
legend('Core Temperature','Location','best');
set(gca,'FontSize',14);
linkaxes(h_ax1,'x');

function x_dot = Dynamics(~, x, u, model)
Vc = x(2);
Tc = x(3);
Ts = x(4);

R1  = getParamESC('RParam',Tc,model);  % RC circuit resistance
tau = getParamESC('RCParam',Tc,model); % RC time constant
C1 = tau/R1;                            % RC circuit capacitance

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

function [y, OCV] = Output(x, u, model, coe)
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