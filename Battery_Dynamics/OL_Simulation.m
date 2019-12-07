clc; clear; close all

% Load all electrical parameters
load('SAMmodel.mat')

dt = 0.001;      % sampling time
alpha0 = 3.613;  % yintercept of SOC-OCV approximation
alpha1 = 0.4631; % slope of SOC-OCV curve between 20% and 80% SOC

R0  = getParamESC('R0Param',25,model); % Ohmic resistance
R1  = getParamESC('RParam',25,model);  % RC circuit resistance
tau = getParamESC('RCParam',25,model); % RC time constant
C1 = tau/R1;                           % RC circuit capacitance
Q = getParamESC('QParam',25,model);    % Battery capacity

% Discrete electrical dynamics -> augmented with one extra row to account
% for the affine portion of the SOC-OCV curve
A = [   1,                0, 0;
        0, exp(-dt/(R1*C1)), 0;
        0,                0, 1];
    
B = [                   -dt/Q;
      R1*(1-exp(-dt/(R1*C1)));
                            0];
  
C = [alpha1, -1, alpha0;
          0,  0,      1];
  
D = [-R0;0];

sys_d = ss(A,B,C,D);

% Set parameters for the MPC problem
Np = 10;   % prediction horizon
Nc = 3;    % control horizon
r = 1e-04; % control weighting on deltaCurrent
SP = 0.8;  % SOC set point

% initial conditions of the extended state
X0 = [0.2; 0; 1; 0]; % [z; Vc; 1; i]

% set the constraints
lN = 100; % large number for states that don't need to be constrained

% Q and -Q are used for the current constraints indicating a 1C
% charge/discharge rate meaning the battery will be fully
% discharged/charged in 1 hour
xlimit.max = [1; lN; 1; Q];    % [z, Vc, 1, i] (not sure about the 1)
xlimit.min = [0; -lN; 1; -Q];

% we have no constraints on deltaU but HW solutions still have it input
% into the constraint formulation so I'll do the same.
delta_ulimit.max = 5;
delta_ulimit.min = -delta_ulimit.max; 

% constrain the terminal voltage with manufacturer's specs which are just
% open circuit voltage at 0% SOC and 100% SOC at 25 deg C
OCV_low = GetOCV(0,25,model);  % OCV which corresponds to 0% SOC 
OCV_high = GetOCV(1,25,model); % OCV which corresponds to 100% SOC
ylimit.max = [OCV_high; 1];    % [Vt, 1] (same doubts about the 1)
ylimit.min = [OCV_low; 1]; 

[H,q,G,W,T,Gamma,SP,Phi_mult_Atilde,IMPC,Atilde,Btilde,Ctilde] = ...
    formQPMatrices(A, B, C, D, X0, Np, Nc, r, SP, xlimit, ylimit,delta_ulimit);

%Wtilde = W + T*X0;

lambda0 = ones(size(G,1),1);


Nsim = 25;
X = X0;
u0 = 0; % Initial DeltaU is zero
u = u0;
y0 = zeros(size(C));
Y = y0;

Output = [];
States = [];

for i = 1:Nsim
 
    Wtilde = W + T*X0;
    
    [DeltaU, lambda] = myQP(H, q, G, Wtilde, lambda0);
    lambda0 = lambda;
    
    [U,~,~,~,lam] = quadprog(H,q,G,Wtilde);

    du = IMPC*DeltaU; % delta u
    u = du + u;       % actual control - 4th state in the extended state vector
   
    X(:,i+1) = Atilde*X(:,i) + Btilde*u;
    Y(:,i)   = Ctilde*X(:,i);

    X0 = X(:,end);
   
    States = [States, X(:,i)];
    Output = [Output, Y(:,i)];
     
end

plot((1:1:(Nsim+1)),X(1,:))


% % plotting
% x = zeros(4, 50);
% x(:,1) = [0.2; 0; 1; -1];
% y = zeros(2, 50);
%
% Aaug = [A,B;zeros(1,3),1];
% Baug = [zeros(3,1);1];
% Caug = [C,D];
% 
% for i = 1:50
%    x(:,i+1) = Aaug*x(:,i) + Baug*(0);
%    y(:,i)   = Caug*x(:,i);
% end
% 
% plot(x(1,:))
