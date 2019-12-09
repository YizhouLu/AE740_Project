clc; clear; close all

% Load all electrical parameters
load('SAMmodel.mat')

Temp = 25; % degC
SOC = model.SOC;
OCV = model.OCV0 + Temp*model.OCVrel;

dt = 0.001;      % sampling time
alpha0 = 3.613;  % yintercept of SOC-OCV approximation
alpha1 = 0.4631; % slope of SOC-OCV curve between 20% and 80% SOC

R0  = getParamESC('R0Param',Temp,model); % Ohmic resistance
R1  = getParamESC('RParam',Temp,model);  % RC circuit resistance
tau = getParamESC('RCParam',Temp,model); % RC time constant
C1 = tau/R1;                           % RC circuit capacitance
Q = getParamESC('QParam',Temp,model);    % Battery capacity

%% Discrete electrical dynamics
% state = [delta_z; delta_Vc; i_last; e; z; Vc; 1]
A = [   1,                0, 0, 0, 0, 0, 0;
        0, exp(-dt/(R1*C1)), 0, 0, 0, 0, 0;
        0,                0, 1, 0, 0, 0, 0;
        1,                0, 0, 1, 0, 0, 0;
        1,                0, 0, 0, 1, 0, 0;
        0,                1, 0, 0, 0, 1, 0;
        0,                0, 0, 0, 0, 0, 1];
% input = delta_i
B = [                   -dt/Q;
      R1*(1-exp(-dt/(R1*C1)));
                            1;
                            0;
                            0;
                            0;
                            0];
% output = [e, Vt, i, z]                  
C = [0, 0,   0, 1,      0,  0,      0;
     0, 0, -R0, 0, alpha1, -1, alpha0;
     0, 0,   1, 0,      0,  0,      0;
     0, 0,   0, 0,      1,  0,      0];
  
D = [0; -R0; 1; 0];

sys_d = ss(A,B,C,D);

N = struct();
N.prediction = 5;
N.control = 3;

penalty = struct;
penalty.Q = diag([1.5, 0, 0, 0]);
penalty.R = 0.5; 

limit = struct();
limit.y.max = [ 100; GetOCV(0.8, Temp, model);  0; 0.8];
limit.y.min = [-100; GetOCV(0.2, Temp, model); -Q; 0.2];
limit.du.max = 100;
limit.du.min = -100;

[H, L, G, W, T] = formQPMatrices(sys_d, N, penalty, limit);

% initial conditions of the extended state
%    [dz; dVc; i_prev;    e;   z; Vc; 1]
X0 = [ 0;   0;      0; -0.6; 0.2;  0; 1]; 

du = 0; % Initial DeltaU is zero
y0 = C*X0 + D*du;
Y = y0;


SOC_setpoint = 0.8;
lambda = ones(size(G,1),1);

Nsim = 1500;
X = zeros(7, Nsim); X(:,1) = X0;
U = zeros(1, Nsim);

for i = 1:Nsim
 
    X0(4) = X0(5) - SOC_setpoint;
    
    Wtilde = W + T*X0;
    q = L*X0;
    
    [dU, lambda] = myQP(H, q, G, Wtilde, lambda, 30);
    
    
    du = dU(1);     % delta u
    u = du + X0(3); % actual control actuation
    U(1,i) = u;

    X(:,i+1) = A*X(:,i) + B*du; % [delta_z; delta_Vc; i_(k - 1); e; z; Vc; 1]
    Y(:,i)   = C*X(:,i) + D*du; % [e, Vt, i, z]

    X0 = X(:,i+1);
end

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
