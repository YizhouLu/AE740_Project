%% Post Simulation Processing

% Extract simulation data from "To Workspace" blocks

% electrical outputs
sim_time = out.tout;
Vt_model = out.Vt;
OCV_out = out.OCV;
SOC_out = out.SOC;
HeatGen = out.HeatGen;

% thermal outputs
HeatGenCurrentProfile = out.HeatGen_current_profile;
Ts = out.Ts;
Tc = out.Tc;

% Plot the electrical outputs
figure('Name','Equivalent Circuit Model: Electrical Output');
set(gcf,'Color','White','Units','Normalized','Position',[0.2 0.2 0.6 0.6]);

h_ax1(1) = subplot(221);
plot(sim_time,Vt_model,'b','LineWidth',2);
hold on
if set_profile == 0
    plot(t,Vt_experiment,'--r','LineWidth',2);
end
ylabel('Voltage [V]');
legend('Model Vt','Experimental Vt','Location','best');
grid on
set(gca,'FontSize',14);

h_ax1(2) = subplot(222);
plot(sim_time,OCV_out,'b','LineWidth',2);
ylabel('Voltage [V]');
legend('Open Circuit Voltage','Location','best');
grid on;
set(gca,'Fontsize',14);

h_ax1(3) = subplot(224);

if set_point == 0
    plot(t,current,'b','LineWidth',2)
elseif set_point == 1
    plot(sim_time,charging_current,'b','LineWidth',2)
else
    plot(sim_time,HeatGenCurrentProfile,'b','LineWidth',2)
end

legend('Current Profile','Location','best');
xlabel('Time [s]');
grid on;
sgtitle('Electrical Model');
set(gca,'Fontsize',14);
linkaxes(h_ax1,'x');

h_ax1(4) = subplot(224);
plot(sim_time,SOC_out,'b','LineWidth',2);
legend('State of Charge','Location','best');
xlabel('Time [s]');
grid on;
sgtitle('Electrical Model');
set(gca,'Fontsize',14);
linkaxes(h_ax1,'x');

% Plot the thermal outputs
figure('Name','Equivalent Circuit Model: Thermal Output');
set(gcf,'Color','White','Units','Normalized','Position',[0.2 0.2 0.6 0.6]);

h_ax2(1) = subplot(211);
plot(sim_time,Ts,'b','LineWidth',2);
hold on
plot(sim_time,Tc,'--r','LineWidth',2);
ylabel('[deg C]');
legend('Surface Temperature','Core Temperature','Location','best');
grid on
set(gca,'FontSize',14);

h_ax2(2) = subplot(212);
plot(sim_time,HeatGen,'b','LineWidth',2);
xlabel('Time [s]');
ylabel('Heat [W]');
legend('Heat Generation (Joule Heating)','Location','best');
grid on;
sgtitle('Thermal Model');
set(gca,'Fontsize',14);
linkaxes(h_ax2,'x');