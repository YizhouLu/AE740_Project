%% Initialization Script

clc;clear;close

% SAMmodel.mat is used to construct our own Equivalent Circuit Model. Based
% on the capacity and open circuit voltage limits, this cell is cylindrical
% which we need to implement the thermal model.
load('SAMmodel.mat');
load('SAM_DYN_15_P25'); % experimental dynamic testing data at 25 C

% SOC - OCV Lookup Table -> OCV(z(t),T(t)) = OCV0 + T(t)*OCVrel(z(t))
OCV0   = model.OCV0;
OCVrel = model.OCVrel;
SOC    = model.SOC;    % useful for the GetOCV function

% R0, R1 and C1 Parameter Lookup Tables
R0  = model.R0Param; % Ohms
R1  = model.RParam;  % Ohms
tau = model.RCParam; % Sec
C1 = tau./R1;        % Farad

% temperatures at which Lookup Tables are implemented
temperatures = model.temps;

% use the average of the capacity since very little deviation
Q = mean(model.QParam); % Ah

% use experimental data to validate the model
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

discharge_current_profile = timeseries(current,t);

set_profile = 0;
if set_profile == 0
    z0 = 1;
elseif set_profile == 1
    z0 = 0.2;
else 
    z0 = 0.5;
end

% Define initial conditions for the electrical states
Vc0 = 0;

% Define thermal model parameters
Cc = 80;   % J/K  (tune)
Cs = 10;   % J/K  (tune)
Re = 0.05; % Ohms (combo of R1 and R0)
Rc = 1;    % K/W  (tune)
Ru = 4;    % K/W  (tune)
Tamb = 25; % degC (keep ambient at 25 to start -> if we choose to add
           %       cell to cell conduction this will change)
           
% Define initial conditions for the thermal states
Tc0 = 25;  % degC (initial cell core temperature)
Ts0 = 25;  % degC (initial cell surface temperature)

% Put the thermal model into State Space form 

% state dynamics
A = [-1/(Rc*Cc), 1/(Rc*Cc);
    1/(Cs*Rc), -1/(Cs*Rc) - 1/(Cs*Ru)];

% 2 input system -> heat generation and ambient temperature
B = [1/Cc, 0;
    0, 1/(Cs*Ru)];

% output both core and surface temp -> we will constrain Tc in the MPC
% formulation instead of Ts because it is worst case scenario
C = [1,0;
    0 1]; 

D = [0 0;0 0]; % no direct feedthrough term
