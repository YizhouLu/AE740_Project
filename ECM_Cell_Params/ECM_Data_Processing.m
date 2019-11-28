%% Post Simulation Processing

% Extract simulation data from "To Workspace" blocks
sim_time = out.tout;
Vt_model = out.terminal_voltage;
OCV_out = out.OCV;
SOC_out = out.SOC;

figure('Name','Equivalent Circuit Model Output');
set(gcf,'Color','White','Units','Normalized','Position',[0.2 0.2 0.6 0.6]);

h_ax(1) = subplot(311);
plot(sim_time,Vt_model,'b','LineWidth',2);
hold on
plot(t,Vt_experiment,'--r','LineWidth',2);
legend('Model Vt','Experimental Vt','Location','best');
grid on
set(gca,'FontSize',14);

h_ax(2) = subplot(312);
plot(sim_time,OCV_out,'b','LineWidth',2);
legend('Open Circuit Voltage','Location','best');
grid on;
set(gca,'Fontsize',14);

h_ax(3) = subplot(313);
plot(sim_time,SOC_out,'b','LineWidth',2);
legend('State of Charge','Location','best');
xlabel('Time [s]');
grid on;
set(gca,'Fontsize',14);
linkaxes(h_ax,'x');