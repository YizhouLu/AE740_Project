clc; clear; close all

load('SAMmodel.mat')

dt = 0.001;      % sampling time
alpha0 = 3.613;  % yintercept of OCV-SOC approximation
alpha1 = 0.4631; % slope of SOC-OCV curve between 20% and 80% SOC

R0  = getParamESC('R0Param',25,model); % Ohmic resistance
R1  = getParamESC('RParam',25,model);  % RC circuit resistance
tau = getParamESC('RCParam',25,model); % RC time constant
C1 = tau/R1;                           % RC circuit capacitance
Q = getParamESC('QParam',25,model);    % Battery Capacity


A = [   1,                0, 0;
        0, exp(-dt/(R1*C1)), 0;
        0,                0, 1];
    
B = [                   -dt/Q;
      R1*(1-exp(-dt/(R1*C1)));
                            0];
  
C = [alpha1, -1, alpha0;
          0,  0,      1];
  
D = [-R0;0];

Aaug = [A,B;zeros(1,3),1];
Baug = [zeros(3,1);1];
Caug = [C,D];

Np = 10;   % prediction horizon
Nc = 1;    % control horizon
r = 1e-04; % control weighting
SP = 0.8;  % SOC set point

% initial conditions of the extended state
X0 = [0.2; 0; 1; -1]; % [z, Vc, 1, i]

lN = 100;
xlimit.max = [SP, lN, 1, Q]; % [z, Vc, 1, i]
xlimit.min = -xlimit.max;

OCV_low = GetOCV(0,25,model);
OCV_high = GetOCV(1,25,model);

ylimit.max = [OCV_high;1];
ylimit.min = [OCV_low;1]; 

[H,q,G,W,T] = formQPMatrices(A, B, C, D, X0, Np, Nc, r, SP, xlimit, ylimit);





% plotting

% x = zeros(4, 50);
% x(:,1) = [0.2; 0; 1; -1];
% y = zeros(2, 50);
% 
% for i = 1:50
%    x(:,i+1) = Aaug*x(:,i) + Baug*(0);
%    y(:,i)   = Caug*x(:,i);
% end
% 
% plot(x(1,:))
