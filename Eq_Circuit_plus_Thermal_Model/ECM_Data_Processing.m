%% Post Simulation Processing

% Extract simulation data from "To Workspace" blocks

% electrical outputs
sim_time = out.tout;
Vt_model = out.terminal_voltage;
OCV_out = out.OCV;
SOC_out = out.SOC;
HeatGen = out.HeatGen;

% thermal outputs
Ts = out.Ts;
Tc = out.Tc;

% Plot the electrical outputs
figure('Name','Equivalent Circuit Model: Electrical Output');
set(gcf,'Color','White','Units','Normalized','Position',[0.2 0.2 0.6 0.6]);

h_ax1(1) = subplot(311);
plot(sim_time,Vt_model,'b','LineWidth',2);
hold on
plot(t,Vt_experiment,'--r','LineWidth',2);
legend('Model Vt','Experimental Vt','Location','best');
grid on
set(gca,'FontSize',14);

h_ax1(2) = subplot(312);
plot(sim_time,OCV_out,'b','LineWidth',2);
legend('Open Circuit Voltage','Location','best');
grid on;
set(gca,'Fontsize',14);

h_ax1(3) = subplot(313);
plot(sim_time,SOC_out,'b','LineWidth',2);
legend('State of Charge','Location','best');
xlabel('Time [s]');
grid on;
set(gca,'Fontsize',14);
linkaxes(h_ax1,'x');

% Plot the thermal outputs
figure('Name','Equivalent Circuit Model: Thermal Output');
set(gcf,'Color','White','Units','Normalized','Position',[0.2 0.2 0.6 0.6]);

h_ax2(1) = subplot(211);
plot(sim_time,Ts,'b','LineWidth',2);
hold on
plot(sim_time,Tc,'--r','LineWidth',2);
legend('Surface Temperature','Core Temperature','Location','best');
grid on
set(gca,'FontSize',14);

h_ax2(2) = subplot(212);
plot(sim_time,HeatGen,'b','LineWidth',2);
xlabel('Time [s]');
ylabel('Heat [W]');
legend('Heat Generation (Joule Heating)','Location','best');
grid on;
set(gca,'Fontsize',14);
linkaxes(h_ax2,'x');