clc;clear;close all

load('SAMmodel.mat');
load('control_jojo.mat');
U = Control;
Temp = 25;

Q = getParamESC('QParam',Temp,model);    % Battery capacity

dt = 1;

Nsim = 1500; 

sys_d = generateElecModel_test(Temp, model, dt);
tic
X(:,1) = [0;0;3.613];
t = 0;
x_curr = [0.2;0];
Traj = x_curr;  
for i = 1:Nsim
    [t_ode45, x_ode45] = ode45(@(t,x) electricalDynamics(t, x, U(i), model), t + dt/10 : dt/10 : t + dt, x_curr);
    t = t_ode45(end);
    x_curr = x_ode45(end,:)';
    Traj = [Traj, x_curr];
%     X(:,i+1) = sys_d.A * X(:,i) + sys_d.B * -Q;
%     Y(:,i)   = sys_d.C * X(:,i) + sys_d.D * -Q;
end
toc
figure(1)
subplot(1,2,1); hold on; grid on
plot((0:Nsim)*dt, Traj(1,:))
subplot(1,2,2); hold on; grid on
plot((0:Nsim)*dt, Traj(2,:))

% figure(1)
% subplot(1,3,1); hold on; grid on
% plot((0:Nsim)*dt, X(1,:))
% subplot(1,3,2); hold on; grid on
% plot((0:Nsim)*dt, X(2,:))
% subplot(1,3,3); hold on; grid on
% plot((0:Nsim)*dt, X(3,:))

function x_dot = electricalDynamics(t, x, u, model)
Temp = 25;
alpha0 = 3.613;  % yintercept of SOC-OCV approximation
alpha1 = 0.4631; % slope of SOC-OCV curve between 20% and 80% SOC
R0  = getParamESC('R0Param',Temp,model); % Ohmic resistance
R1  = getParamESC('RParam',Temp,model);  % RC circuit resistance
tau = getParamESC('RCParam',Temp,model); % RC time constant
C1 = tau/R1;                             % RC circuit capacitance
Q = getParamESC('QParam',Temp,model);    % Battery capacity

x_dot = zeros(size(x));
x_dot(1,1) = -1/(Q*3600) * u;
x_dot(2,1) = -1/(R1*C1)*x(2) + 1/C1 * u;
end