close all
load('MAP.mat');
load('SpeedProfile.mat');
load('SAMmodel.mat');
load('SOC_OCV_coe.mat');

%% Simulation parameters
Nsim = 3000;

%% Figure parameters
figure
subplot(1,2,1)
plot(PATH(1,:),PATH(2,:),'k','linewidth',2);axis equal;grid on;hold on

subplot(1,2,2)
xlabel('Time, s'); title('Bus1 State of Charge'); hold on; grid on; 
xlim([0 Nsim]); ylim([0 1])

%% Bus stop location
BusStop_idx = [1, 243246];
for j = 1:length(BusStop_idx)
    subplot(1,2,1)
    scatter(PATH(1,BusStop_idx(j)),PATH(2,BusStop_idx(j)),'b','filled')
end

%% Simulation initialization
Init.Distance     = 0;
Init.Position_idx = 1;
Init.isRunning    = 1;
Init.dz     = 0; 
Init.dVc    = 0;
Init.SOC    = 1;
Init.Vc     = 0;
Init.i_last = 0;
Init.e      = 0;
Init.dTc    = 0;
Init.dTs    = 0;
Init.Tc     = 25;
Init.Ts     = 25;
Bus1 = Bus(Init);

k = 1;

for i = 1:Nsim
    Bus1.updatePosition(PATH, VELOCITY, i);
    Bus1.updateSOC();
    Bus1.updateState(BusStop_idx);
    
    if Bus1.isRunning == 0
        [Bus1.X, y, ocv, u] = MPC(Bus1.X, model, coe);
        Y(:,k) = y;
        k = k + 1;
    end
    
    Bus1.graph(i, 1, 2);
    drawnow;
    delete(Bus1.graph_handle);
end

