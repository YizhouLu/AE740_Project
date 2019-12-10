clc; clear; close all

% Load all electrical parameters
load('SAMmodel.mat')

Temp = 25; % degC
SOC = model.SOC;
OCV = model.OCV0 + Temp*model.OCVrel;

%% Discrete electrical dynamics
N = struct();
N.prediction = 5;
N.control = 3;

penalty = struct();
penalty.Q = diag([1.5, 0, 0, 0, 0]); % output = [e, Vt, i, z, Tc]
penalty.R = 0.5;                     % input  = di

limit = struct();
Vt_max = GetOCV(0.8, Temp, model);
Vt_min = GetOCV(0.2, Temp, model);
i_min = -getParamESC('QParam',Temp,model);
limit.y.max = [ 100; Vt_max;     0; 0.8; 50];
limit.y.min = [-100; Vt_min; i_min; 0.2;  0];
limit.du.max =  100;
limit.du.min = -100;

%    [dz; dVc; i_prev;    e;   z; Vc; 1; dTc; dTs; Tc_prev; Ts_prev]
X0 = [0;    0;      0; -0.6; 0.2;  0; 1; 0;   0;   25;      25];
%    di
du = 0;

SOC_setpoint = 0.8;
dt = 0.001;      % sampling time

Nsim = 1500;
X = zeros(11, Nsim); X(:,1) = X0;
U = zeros( 1, Nsim);
    
for i = 1:Nsim
    [A_elec, B_elec, C_elec, D_elec] = generateElecModel(Temp, model, dt);
    [A_thml, B_thml, C_thml, D_thml] = generateThmlModel(Temp, X0(3), model, dt);
    
    A_aug = blkdiag(A_elec, A_thml);
    B_aug = [B_elec;B_thml];
    C_aug = blkdiag(C_elec, C_thml);
    D_aug = [D_elec;D_thml];
    sys_d = ss(A_aug, B_aug, C_aug, D_aug);
    
    [H, L, G, W, T] = formQPMatrices(sys_d, N, penalty, limit);
    
    if i == 1
        lambda = ones(size(G,1),1);
    end

    
    X0(4) = X0(5) - SOC_setpoint;
    
    Wtilde = W + T*X0;
    q = L*X0;
    
    [dU, lambda] = myQP(H, q, G, Wtilde, lambda, 100);
      
    du = dU(1);     % delta u
    u = du + X0(3); % actual control actuation
    U(1,i) = u;
    
    X(:,i+1) = A_aug*X(:,i) + B_aug*du; % [delta_z; delta_Vc; i_(k - 1); e; z; Vc; 1]
    Y(:,i)   = C_aug*X(:,i) + D_aug*du; % [e, Vt, i, z]
    
    X0 = X(:,i+1);
    Temp = Y(5,i);
end
%% Plot results
close all
t = 1:1:Nsim; % sampling instant

figure('Name','Fast Charge Output');
set(gcf,'Color','White','Units','Normalized','Position',[0.2 0.2 0.6 0.6]);

h_ax1(1) = subplot(221); % error
plot(t,Y(1,:),'b','LineWidth',2);
% no constraints on error so not plotted
xlabel('Time [s]');
legend('error','Location','best');
set(gca,'FontSize',14);

h_ax1(2) = subplot(222); % voltage
plot(t,Y(2,:),'b','LineWidth',2);
hold on
plot(t,limit.y.max(2)*ones(size(t)),'--r','LineWidth',2);
plot(t,limit.y.min(2)*ones(size(t)),'--r','LineWidth',2);
xlabel('Time [s]');
legend('terminal voltage','Location','best');
set(gca,'FontSize',14);

h_ax1(1) = subplot(223); % current
plot(t,Y(3,:),'b','LineWidth',2);
hold on
plot(t,limit.y.max(3)*ones(size(t)),'--r','LineWidth',2);
plot(t,limit.y.min(3)*ones(size(t)),'--r','LineWidth',2);
xlabel('Time [s]');
legend('current','Location','best');
set(gca,'FontSize',14);

h_ax1(1) = subplot(224); % SOC
plot(t,Y(4,:),'b','LineWidth',2);
hold on
plot(t,limit.y.max(4)*ones(size(t)),'--r','LineWidth',2);
plot(t,limit.y.min(4)*ones(size(t)),'--r','LineWidth',2);
xlabel('Time [s]');
legend('State of Charge','Location','best');
set(gca,'FontSize',14);
linkaxes(h_ax1,'x');

figure(2)
plot(t,Y(5,:),'b','LineWidth',2);grid on

