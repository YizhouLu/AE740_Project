%% Initialization Script

clc;clear;close

% ATLmodel.mat is a bunch of open source test data we will use to construct
% our own Equivalent Circuit Model. I chose this cell just because it has a
% really high capacity but maybe we want to look at the other options to
% see which fits our needs the most. No code would change except the line
% below which loads model parameters.
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

% use experimental data to validate the model
load('Dynamic_data.mat')
time = DYNData.script1.time; 
current = DYNData.script1.current;
Vt_experiment = DYNData.script1.voltage;

% get rid of duplicate time steps
ind = find(diff(time)<=0);
time(ind+1) = [];
Vt_experiment(ind+1) = [];
current(ind+1) = [];

tstart = time(1);
tfinal=time(end);
deltaT = 1; % one-second sampling
t = (tstart:deltaT:tfinal) - tstart;

current = interp1(time,current,tstart:deltaT:tfinal);
Vt_experiment = interp1(time,Vt_experiment,tstart:deltaT:tfinal);

current_profile = timeseries(current,t);

% Define initial conditions for the electrical states
z0  = 1;
Vc0 = 0;

% Define initial conditions for the thermal states
Cc = 80;   % J/K  (tune)
Cs = 10;   % J/K  (tune)
Re = 0.05; % Ohms (combo of R1 and R0)
Rc = 1;    % K/W  (tune)
Ru = 4;    % K/W  (tune)
Tamb = 25; % degC (keep ambient at 25 to start -> if we choose to add
           %       cell to cell conduction this will change)
Tc0 = 25;  % degC (initial cell core temperature)
Ts0 = 25;  % degC (initial cell surface temperature)

% Put the thermal model into State Space form since easier to visualize

% state dynamics
A = [-1/(Rc*Cc), 1/(Rc*Cc);
    1/(Cs*Rc), -1/(Cs*Rc) - 1/(Cs*Ru)];

% 2 input system -> heat generation and ambient temperature
B = [Re/Cc, 0;
    0, 1/(Cs*Ru)];

% output both core and surface temp -> will constraint core temperature
% because it is worst case scenario
C = [1,0;
    0 1]; 

D = [0;0];
