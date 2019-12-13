clc; clear; close all

load('SAMmodel.mat')
load('SOC_OCV_coe.mat')
%% Model initial conditions
%    [dz; dVc;   z; Vc; 1; i_last;    e; dTc; dTs; Tc; Ts]
X0 = [ 0;   0; 0.2;  0; 1;      0; -0.7;   0;   0; 25; 25];

Nsim = 1000;

X = zeros(11, Nsim+1); X(:,1) = X0;
U = zeros( 1, Nsim);
Y = zeros( 5, Nsim);
OCV = zeros(1, Nsim);

for i = 1:Nsim
    [X_next, y, ocv, u] = MPC(X0, model, coe);
    X0 = X_next;
    
    U(1,i) = u;
    X(:,i+1) = X_next;
    Y(:,i) = y;
    OCV(:,i) = ocv;
end



%% Plot results
limit = struct();
% Vt_max = GetOCV(0.8, Temp, model);
Vt_max = 4.2; % GetOCV(0.8, Temp, model);
Vt_min = 3.0; % GetOCV(0.2, Temp, model);
i_min = -getParamESC('QParam',25,model);
limit.y.max = [Vt_max;  100;     0; 0.9; 50];
limit.y.min = [Vt_min; -100; i_min; 0.2;  0];
limit.du.max =  100;
limit.du.min = -100;
% close all
t = (1:1:Nsim)*1;

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

% function x_dot = Dynamics(~, x, u, model)
% Vc = x(2);
% Tc = x(3);
% Ts = x(4);
% 
% R1  = getParamESC('RParam',Tc,model);  % RC circuit resistance
% tau = getParamESC('RCParam',Tc,model); % RC time constant
% C1 = tau/R1;                             % RC circuit capacitance
% Q = getParamESC('QParam',Tc,model);    % Battery capacity
% 
% Cc = 80;   % J/K  (tune)
% Cs = 10;   % J/K  (tune)
% Re = 2;    % Ohms (combo of R1 and R0)
% Rc = 1;    % K/W  (tune)
% Ru = 4;    % K/W  (tune)
% 
% Tamb = 25;
% 
% z_dot  = -1/(Q*3600) * u;
% Vc_dot = -1/(R1*C1) * Vc + 1/C1 * u;
% 
% Tc_dot = Re/Cc * u^2 - (Tc - Ts)/(Cc * Rc);
% Ts_dot = (Tc - Ts)/(Cs * Rc) - (Ts - Tamb)/(Cs * Ru);
% 
% x_dot = [z_dot;Vc_dot;Tc_dot;Ts_dot];
% end
% 
% function [y, OCV] = Output(x, u, model, coe)
% z  = x(3);
% Vc = x(4);
% e  = x(7);
% Tc = x(10);
%   
% OCV = polyval(coe, z);
% R0  = getParamESC('R0Param',Tc,model); % Ohmic resistance
% 
% Vt = OCV - R0*u - Vc;
% 
% y = [Vt;e;u;z;Tc];
% end