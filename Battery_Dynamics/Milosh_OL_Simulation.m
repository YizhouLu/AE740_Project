clc; clear; close all

% Load all electrical parameters
load('SAMmodel.mat')

Temp = 25; % degC
SOC = model.SOC;
OCV = model.OCV0 + Temp*model.OCVrel;

dt = 0.001;      % sampling time
alpha0 = 3.613;  % yintercept of SOC-OCV approximation
alpha1 = 0.4631; % slope of SOC-OCV curve between 20% and 80% SOC

R0  = getParamESC('R0Param',25,model); % Ohmic resistance
R1  = getParamESC('RParam',25,model);  % RC circuit resistance
tau = getParamESC('RCParam',25,model); % RC time constant
C1 = tau/R1;                           % RC circuit capacitance
Q = getParamESC('QParam',25,model);    % Battery capacity

% Discrete electrical dynamics

% [delta_z; delta_Vc; i_(k - 1); e; z; Vc; 1]
A = [   1,                0, 0, 0, 0, 0, 0;
        0, exp(-dt/(R1*C1)), 0, 0, 0, 0, 0;
        0,                0, 1, 0, 0, 0, 0;
        1,                0, 0, 1, 0, 0, 0;
        1,                0, 0, 0, 1, 0, 0;
        0,                1, 0, 0, 0, 1, 0;
        0,                0, 0, 0, 0, 0, 1];
    
B = [                   -dt/Q;
      R1*(1-exp(-dt/(R1*C1)));
                            1;
                            0;
                            0;
                            0;
                            0];

% [e, Vt, i, z]                  
C = [0, 0,   0, 1,      0,  0,      0;
     0, 0, -R0, 0, alpha1, -1, alpha0;
     0, 0,   1, 0,      0,  0,      0;
     0, 0,   0, 0,      1,  0,      0];
  
D = [0; -R0; 1; 0];

sys_d = ss(A,B,C,D);

% Set parameters for the MPC problem
Np = 10;  % prediction horizon
Nc = 1;   % control horizon
r  = 0.5; % control weighting on deltaCurrent

% set the constraints
lN = 100; % large number for outputs that don't need to be constrained

% we have no constraints on deltaU but HW solutions still have it input
% into the constraint formulation so I'll do the same.
delta_ulimit.max = lN;
delta_ulimit.min = -delta_ulimit.max;

% constrain the terminal voltage with manufacturer's specs which are just
% open circuit voltage at 0% SOC and 100% SOC at 25 deg C
OCV_low = GetOCV(0.2,Temp,model);  % OCV which corresponds to 0.2% SOC 
OCV_high = GetOCV(0.8,Temp,model); % OCV which corresponds to 80% SOC

% Q and -Q are used for the current constraints indicating a 1C
% charge/discharge rate meaning the battery will be fully
% discharged/charged in 1 hour

% extended output: [e, Vt, i, z]
ylimit.max = [ lN; OCV_high; 0; 0.8];
ylimit.min = [-lN; OCV_low; -Q; 0.2];

[H,L,G,W,T] = formQPMatrices_LoopMethod(A, B, C, D, Np, Nc, r, ylimit, delta_ulimit);
lambda0 = ones(size(G,1),1);

Nsim = 1500;

% initial conditions of the extended state
% [delta_z; delta_Vc; i_(k - 1); e; z; Vc; 1]
X0 = [0; 0; 0; -0.6; 0.2; 0; 1]; 

X = X0;
delta_u0 = 0; % Initial DeltaU is zero
delta_u = delta_u0;
y0 = C*X0 + D*delta_u0;
Y = y0;

Control = zeros(1,Nsim);

SOC_setpoint = 0.8;

for i = 1:Nsim
 
    X0(4) = X0(5) - SOC_setpoint;
    
    Wtilde = W + T*X0;
    q = L*X0;
    
    %[DeltaU, lambda] = myQP(H, q, G, Wtilde, lambda0);
    [DeltaU,~,~,~,lambda] = quadprog(H,q,G,Wtilde);
    lambda0 = lambda;

    delta_u = DeltaU(1);  % delta u
    u = delta_u + X0(3);  % actual control actuation
    Control(1,i) = u;

    X(:,i+1) = A*X(:,i) + B*delta_u; % [delta_z; delta_Vc; i_(k - 1); e; z; Vc; 1]
    Y(:,i)   = C*X(:,i) + D*delta_u; % [e, Vt, i, z]

    X0 = X(:,end);
    
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
plot(t,ylimit.max(2)*ones(size(t)),'--r','LineWidth',2);
plot(t,ylimit.min(2)*ones(size(t)),'--r','LineWidth',2);
xlabel('Time [s]');
legend('terminal voltage','Location','best');
set(gca,'FontSize',14);

h_ax1(1) = subplot(223); % current
plot(t,Y(3,:),'b','LineWidth',2);
hold on
plot(t,ylimit.max(3)*ones(size(t)),'--r','LineWidth',2);
plot(t,ylimit.min(3)*ones(size(t)),'--r','LineWidth',2);
xlabel('Time [s]');
legend('current','Location','best');
set(gca,'FontSize',14);

h_ax1(1) = subplot(224); % SOC
plot(t,Y(4,:),'b','LineWidth',2);
hold on
plot(t,ylimit.max(4)*ones(size(t)),'--r','LineWidth',2);
plot(t,ylimit.min(4)*ones(size(t)),'--r','LineWidth',2);
xlabel('Time [s]');
legend('State of Charge','Location','best');
set(gca,'FontSize',14);
linkaxes(h_ax1,'x');
