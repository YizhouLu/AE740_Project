clc; clear; close all

load('MAP.mat');
load('SpeedProfile.mat');
load('SAMmodel.mat');
load('SOC_OCV_pack_coe.mat');


global Param

Param.SOC_setpoint      = 0.5;

% Pack configuration to meet 220 kWh (Vnom = 365 V, Qpack = 602 Ah)
Param.num_series        = 96;
Param.num_parallel_cell = 20;
Param.num_strings       = 14;

Param.Cc = 80; % J/K
Param.Cs = 10; % J/K
Param.Re = 1;  % Ohms
Param.Rc = 1;  % K/W
Param.Ru = 4;  % K/W

vehicle = BuildBusModel(model);



%% Simulation parameters

Nsim = 10000;

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
Init.direction    = 0;
Init.isRunning    = 1;
Init.dz     = 0; 
Init.dVc    = 0;
Init.SOC    = 0.51;
Init.Vc     = 0;
Init.i_last = 0;
Init.e      = 0;
Init.dTc    = 0;
Init.dTs    = 0;
Init.Tc     = 25;
Init.Ts     = 25;
Bus1 = Bus(Init);

k = 1;

% initialize the OCV for the first iteration (y0)
% y = [z, Vc, e, Tc]
[y, ocv] = Output_pack(Bus1.X, Bus1.X(6), model, coe); 

prevSpeed      = 0; % vehicle speed [m/s]
prevMotorSpeed = 0;
Distance       = 0;

motor_torque = [];
motor_speed  = [];
actual_speed = [];
BattPower    = [];

for i = 1:Nsim
    
    Bus1.updatePosition(PATH, VELOCITY, i);
    Bus1.updateState(BusStop_idx);
    
    if Bus1.isRunning == 0
        [Bus1.X, y, ocv, u] = MPC_pack(Bus1.X, model, coe);
        if abs(Bus1.SOC - Param.SOC_setpoint) < 0.001
            Bus1.isRunning = 1;
        end
        Y(:,k) = y;
        OCV(:,k) = ocv;
        
    else
        
%         if Bus1.vel < 0.03
%             Bus1.vel = 0;
%         end
        
%         Bus1.vel
        % vehicle simulator requires velocity input as miles/hour [m/s -> mph]
        speed = (Bus1.vel / 1000) * 0.6214 * 3600; 
        
        results = simBus(vehicle, speed, prevSpeed, prevMotorSpeed, Distance);
        
        motor_torque = [motor_torque;results.motorTorque];
        motor_speed  = [motor_speed;results.motorSpeed];
        actual_speed = [actual_speed; results.actualSpeed];
        BattPower    = [BattPower; results.batteryDemand];
        
        prevSpeed      = results.actualSpeed;
        prevMotorSpeed = results.motorSpeed;
        Distance       = results.distance;
        
        PackCurrent = BattPowerToCurrent(results.batteryDemand,Bus1.X,ocv,model);
        
        [x_ode45_next, y, ocv] = Discharge_Pack(Bus1, model, coe, PackCurrent);
         
        Y(:,k)     = y;
        OCV(:,k)   = ocv;
        
        Bus1.X(3) = x_ode45_next(1);  % z_k+1
        Bus1.SOC  = x_ode45_next(1);  % z_k+1
        
        Bus1.Vc  = x_ode45_next(2);  % Vc_k+1
        Bus1.Tc  = x_ode45_next(3);  % Tc_k+1
        Bus1.Ts  = x_ode45_next(4);  % Ts_k+1

%         Bus1.updatetemp(Bus1.X(4),Bus1.X(10),Bus1.X(11))
        Bus1.updateSOC(Bus1.X(3));
                
%         % Estimate vehicle range
%         range = ((vehicle.drivetrain.pack.socFull - vehicle.drivetrain.pack.socEmpty) / ...
%             (vehicle.drivetrain.pack.socFull - (Bus1.X(3)) * 100)) * Total_Dist


    end
    
    Y(5,k)
    k = k + 1;
    
    Bus1.graph(i, 1, 2);
    drawnow;
    delete(Bus1.graph_handle);
    
end

