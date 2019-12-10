clc; clear; close all

load('SAMmodel.mat')

Temp = 25; % C

N = struct();
N.prediction = 10;
N.control = 1;

penalty = struct();
penalty.Q = diag([0, 1.5, 0, 0, 0]); % output = [Vt, e, i, z, Tc]
penalty.R = 0.5;                     % input  = di

limit = struct();
Vt_max = GetOCV(0.8, Temp, model);
Vt_min = GetOCV(0.2, Temp, model);
i_min = -getParamESC('QParam',Temp,model);
limit.y.max = [Vt_max;  100;     0; 0.8; 100];
limit.y.min = [Vt_min; -100; i_min; 0.2;-100];
limit.du.max =  100;
limit.du.min = -100;

%    [dz; dVc;   z; Vc; 1; i_last;    e; dTc; dTs; Tc_prev; Ts_prev]
X0 = [ 0;   0; 0.2;  0; 1;      0; -0.6;   0;   0;      25;      25];
%    di
du = 0;

SOC_setpoint = 0.8;
dt = 0.1;       % sampling time
t = 0;          % initial time

Nsim = 1000;
X = zeros(11, Nsim+1); X(:,1) = X0;
U = zeros( 1, Nsim);
Y = zeros( 5, Nsim);
tic
for i = 1:Nsim
    [A_elec, B_elec, C_elec, D_elec] = generateElecModel(X0(10), model, dt);
    [A_thml, B_thml, C_thml, D_thml] = generateThmlModel(X0(10), X0(6), model, dt);
    
    A_aug = blkdiag(A_elec, A_thml);
    B_aug = [B_elec;B_thml];
    C_aug = blkdiag(C_elec, C_thml);
    D_aug = [D_elec;D_thml];
    sys_d = ss(A_aug, B_aug, C_aug, D_aug);
    
    [H, L, G, W, T] = formQPMatrices(sys_d, N, penalty, limit);
    
    if i == 1
        lambda = ones(size(G,1),1);
    end
    
    Wtilde = W + T*X0;
    q = L*X0;
    
    [dU, lambda] = myQP(H, q, G, Wtilde, lambda, 50);
      
    du = dU(1);     % delta u
    u = du + X0(6); % actual control actuation
    U(1,i) = u;
    
%     X(:,i+1) = A_aug*X(:,i) + B_aug*du; % [delta_z; delta_Vc; i_(k - 1); e; z; Vc; 1]
%     Y(:,i)   = C_aug*X(:,i) + D_aug*du; % [e, Vt, i, z]
%     
%     X0 = X(:,i+1);
%     Temp = Y(5,i);
    
    x_ode45_curr = [X0(3); X0(4); X0(10); X0(11)]; % z, Vc, Tc, Ts
    
    [t_ode45, x_ode45] = ode45(@(t,x) Dynamics(t, x, u, model), t + dt/10 : dt/10 : t + dt, x_ode45_curr);
    x_ode45_next = x_ode45(end,:)';
    t = t_ode45(end);
    
    X_next = zeros(size(X0));
    X_next(1) = x_ode45_next(1) - X0(3);    % dz_k+1 = z_k+1 - z_k
    X_next(2) = x_ode45_next(2) - X0(4);    % dVc_k+1 = Vc_k+1 - Vc_k
    X_next(3) = x_ode45_next(1);            % z_k+1
    X_next(4) = x_ode45_next(2);            % Vc_k+1
    X_next(5) = 1;
    X_next(6) = u;                          % i_k
    X_next(7) = x_ode45_next(1) - SOC_setpoint; % z_k+1 - 0.8
    X_next(8) = x_ode45_next(3) - X0(10);   % dTc_k+1 = Tc_k+1 - Tc_k
    X_next(9) = x_ode45_next(4) - X0(11);   % dTs_k+1 = Ts_k+1 - Ts_k
    X_next(10) = x_ode45_next(3);           % Tc_k+1
    X_next(11) = x_ode45_next(4);           % Ts_k+1
    
    X(:,i+1) = X_next;
    
    X0 = X_next;
end
toc
%% Plot results
close all
t = (0:1:Nsim)*dt; % sampling instant

figure(1)
plot(t,X(3,:),'b','LineWidth',2);

figure(2)
plot(t,X(10,:),'b','LineWidth',2);

figure(3)
plot(t,X(7,:),'b','LineWidth',2);

% figure('Name','Fast Charge Output');
% set(gcf,'Color','White','Units','Normalized','Position',[0.2 0.2 0.6 0.6]);
% 
% h_ax1(1) = subplot(221); % error
% plot(t,Y(2,:),'b','LineWidth',2);
% % no constraints on error so not plotted
% xlabel('Time [s]');
% legend('error','Location','best');
% set(gca,'FontSize',14);
% 
% h_ax1(2) = subplot(222); % voltage
% plot(t,Y(1,:),'b','LineWidth',2);
% hold on
% plot(t,limit.y.max(1)*ones(size(t)),'--r','LineWidth',2);
% plot(t,limit.y.min(1)*ones(size(t)),'--r','LineWidth',2);
% xlabel('Time [s]');
% legend('terminal voltage','Location','best');
% set(gca,'FontSize',14);
% 
% h_ax1(1) = subplot(223); % current
% plot(t,Y(3,:),'b','LineWidth',2);
% hold on
% plot(t,limit.y.max(3)*ones(size(t)),'--r','LineWidth',2);
% plot(t,limit.y.min(3)*ones(size(t)),'--r','LineWidth',2);
% xlabel('Time [s]');
% legend('current','Location','best');
% set(gca,'FontSize',14);
% 
% h_ax1(1) = subplot(224); % SOC
% plot(t,Y(4,:),'b','LineWidth',2);
% hold on
% plot(t,limit.y.max(4)*ones(size(t)),'--r','LineWidth',2);
% plot(t,limit.y.min(4)*ones(size(t)),'--r','LineWidth',2);
% xlabel('Time [s]');
% legend('State of Charge','Location','best');
% set(gca,'FontSize',14);
% linkaxes(h_ax1,'x');

% figure(2)
% plot(t,Y(5,:),'b','LineWidth',2);grid on

function x_dot = Dynamics(~, x, u, model)
Temp = x(3);

R1  = getParamESC('RParam',Temp,model);  % RC circuit resistance
tau = getParamESC('RCParam',Temp,model); % RC time constant
C1 = tau/R1;                             % RC circuit capacitance
Q = getParamESC('QParam',Temp,model);    % Battery capacity

Cc = 80;   % J/K  (tune)
Cs = 10;   % J/K  (tune)
Re = 0.05; % Ohms (combo of R1 and R0)
Rc = 1;    % K/W  (tune)
Ru = 4;    % K/W  (tune)

z_dot  = -1/(Q*3600) * u;
Vc_dot = -1/(R1*C1)*x(2) + 1/C1 * u;

Tc_dot = Re/Cc * u^2 - (x(3) - x(4))/(Cc * Rc);
Ts_dot = (x(3) - x(4))/(Cs * Rc) - (x(4) - 25)/(Cs * Ru);

x_dot = [z_dot;Vc_dot;Tc_dot;Ts_dot];
end














