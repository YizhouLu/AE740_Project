%% Initialization Script
clc;clear;close

load('ATLmodel.mat');

% SOC - OCV Lookup Table -> OCV(z(t),T(t)) = OCV0 + T(t)*OCVrel(z(t))
OCV0   = model.OCV0;
OCVrel = model.OCVrel;
SOC    = model.SOC;    % probably not needed

% R0, R1 and C1 Parameter Lookup Tables
R0  = (model.R0Param); % Ohms
R1  = (model.RParam);  % Ohms
tau = model.RCParam;   % Sec
C1 = tau./R1;          % Farad

% temperatures at which Lookup Tables are implemented
temperatures = model.temps;

% use the average of the capacity since very little deviation
Q = mean(model.QParam); % Ah

load('Dynamic_data.mat')
time = DYNData.script1.time; 
current = DYNData.script1.current;
Vt_experiment = DYNData.script1.voltage;

ind = find(diff(time)<=0); % get rid of duplicate time steps 
time(ind+1)=[];
Vt_experiment(ind+1)=[];
current(ind+1)=[];

tstart = time(1);
tfinal=time(end);
deltaT = 1;
t = (tstart:deltaT:tfinal) - tstart; % one-second sampling

current = interp1(time,current,tstart:deltaT:tfinal);
Vt_experiment = interp1(time,Vt_experiment,tstart:deltaT:tfinal);

current_profile = timeseries(current,t);

% Define temperature operating point and initial conditions for the states
T = 25;
z0 = 1;
Vc0 = 0;

