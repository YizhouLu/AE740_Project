function vehicle = BuildBusModel(model)

% We use this function at the beginning of the main script to generate a
% series of structures to detail the vehicle properties

% % Setup the Proterra-like vehicle
% Q = getParamESC('QParam',25,model);
% % set up cell: capacity [Ah], weight [g], (vmax, vnom, vmin) [V]
% cell = setupCell(Q, 50, 4.2, 3.8, 3.0); % about 50 grams for an 18650 Li ion cell
% 
% % set up module: numParallel, numSeries, overhead for this cell by weight
% module = setupModule(280, 16, 0.08, cell);
% 
% % set up pack: numSeries, overhead by weight, (full SOC, empty SOC) [%],
% % efficiency for this module
% pack = setupPack(6, 0.1, 100, 0, 0.96, module);
% 
% % set up motor: Lmax [Nm], (RPMrated, RPMmax) [RPM], efficiency,
% % inertia [kg/m2]
% motor = setupMotor(900, 2654, 5500, 0.95, 0.2);
% 
% % set up wheel: radius [m], inertia [kg/m2], rollCoef
% wheel = setupWheel(0.46736, 8, 0.0111); 
% 
% % set up drivetrain: inverter efficiency, fractional regen torque limit,
% % gear ratio, gear inertia [kg/m2], gear efficiency, for this pack,
% % motor, and wheel 
% drivetrain = setupDrivetrain(0.94, 0.9, 3.53, 0.05, 0.97, pack, motor, wheel);
% 
% % set up vehicle: # wheels, roadForce [N], Cd, frontal area [m2], weight
% % [kg], payload [kg], overhead power [W] for this drivetrain
% vehicle = setupVehicle(6, 0, 0.6, 2.46, 26500, 15000, 200, drivetrain);

%%
  % Setup the Chevy Volt-like vehicle
  % set up cell: capacity [Ah], weight [g], (vmax, vnom, vmin) [V]
%   cell = setupCell(15,450,4.2,3.8,3.0);
  cell = setupCell(2.1507, 50, 4.2, 3.8, 3.0);
  % set up module: numParallel, numSeries, overhead for this cell by weight
%   module = setupModule(3,8,0.08,cell);
global Param

num_parallel = Param.num_parallel_cell * Param.num_strings;
  module = setupModule(num_parallel, 16, 0.08, cell);
  % set up pack: numSeries, overhead by weight, (full SOC, empty SOC) [%],
  % efficiency for this module
  pack = setupPack(6, 0.1, 100, 0, 0.96, module);
%   pack = setupPack(12,0.1,75,25,0.96,module);
  % set up motor: Lmax [Nm], (RPMrated, RPMmax) [RPM], efficiency,
  % inertia [kg/m2]
  motor = setupMotor(275,4000,12000,.95,0.2);
  % set up wheel: radius [m], inertia [kg/m2], rollCoef
  wheel = setupWheel(0.35,8,0.0111);
  % set up drivetrain: inverter efficiency, fractional regen torque limit,
  % gear ratio, gear inertia [kg/m2], gear efficiency, for this pack,
  % motor, and wheel 
  drivetrain = setupDrivetrain(0.94,0.9,12,0.05,0.97,pack,motor,wheel);
  % set up vehicle: # wheels, roadForce [N], Cd, frontal area [m2], weight
  % [kg], payload [kg], overhead power [W] for this drivetrain
  vehicle = setupVehicle(4,0,0.22,1.84,1425,75,200,drivetrain);
%%


    % Create a data structure to store cell-specific variables
    function cell = setupCell(capacity,weight,vmax,vnom,vmin)
        cell.capacity       = capacity; % Ah
        cell.weight         = weight;   % grams
        cell.vmax           = vmax;     % V
        cell.vnom           = vnom;     % V
        cell.vmin           = vmin;     % V
        cell.energy         = vnom * capacity; % Watt-hours
        cell.specificEnergy = 1000 * cell.energy / cell.weight; % Wh/kg
    end

    % Create a data structure to store module-specific variables
    function module = setupModule(numParallel,numSeries,overhead,cell)
        module.numParallel    = numParallel;
        module.numSeries      = numSeries;
        module.overhead       = overhead;
        module.cell           = cell;
        module.numCells       = numParallel * numSeries;
        module.capacity       = numParallel * cell.capacity;
        module.weight         = module.numCells*cell.weight * 1/(1-overhead)/1000; % kg
        module.energy         = module.numCells * cell.energy/1000;                % kWh
        module.specificEnergy = 1000 * module.energy / module.weight;           % Wh/kg
    end

    % Create a data structure to store pack-specific variables
    function pack = setupPack(numSeries,overhead,socFull,socEmpty,efficiency,module)
        pack.numSeries  = numSeries;
        pack.overhead   = overhead;
        pack.module     = module;
        pack.socFull    = socFull;
        pack.socEmpty   = socEmpty;   % unitless
        pack.efficiency = efficiency; % unitless, captures I*I*R losses
        pack.numCells   = module.numCells * numSeries;
        pack.weight     = module.weight * numSeries * 1/(1 - overhead); % kg
        pack.energy     = module.energy * numSeries; % kWh
        pack.specificEnergy = 1000 * pack.energy / pack.weight; % Wh/kg
        pack.vmax = numSeries*module.numSeries*module.cell.vmax;
        pack.vnom = numSeries*module.numSeries*module.cell.vnom;
        pack.vmin = numSeries*module.numSeries*module.cell.vmin;
    end

    % Create a data structure to store motor-specific variables
    function motor = setupMotor(Lmax,RPMrated,RPMmax,efficiency,inertia)
        motor.Lmax       = Lmax;     % N-m
        motor.RPMrated   = RPMrated;
        motor.RPMmax     = RPMmax;
        motor.efficiency = efficiency;
        motor.inertia    = inertia; %kg-m2
        motor.maxPower   = 2*pi*Lmax*RPMrated/60000; % kW
    end

    % Create a data structure to store wheel-specific variables
    function wheel     = setupWheel(radius,inertia,rollCoef)
        wheel.radius   = radius;  % m
        wheel.inertia  = inertia; % km-m2
        wheel.rollCoef = rollCoef;
    end

    % Create a data structure to store drivetrain-specific variables
    function drivetrain = setupDrivetrain(inverterEfficiency,regenTorque,...
    gearRatio,gearInertia,gearEfficiency,pack,motor,wheel)

        drivetrain.inverterEfficiency = inverterEfficiency;
        drivetrain.regenTorque        = regenTorque; % fraction of total torque avail.
        drivetrain.pack               = pack;
        drivetrain.motor              = motor;
        drivetrain.wheel              = wheel;
        drivetrain.gearRatio          = gearRatio;
        drivetrain.gearInertia        = gearInertia; % km-m2, measured on motor side
        drivetrain.gearEfficiency     = gearEfficiency;
        drivetrain.efficiency         = pack.efficiency * inverterEfficiency * ...
            motor.efficiency * gearEfficiency;
    end

    % Create a data structure to store all vehicle specifications
    function vehicle = setupVehicle(wheels,roadForce,Cd,A,weight,payload,overheadPwr,drivetrain)
        vehicle.drivetrain  = drivetrain;
        vehicle.wheels      = wheels;      % number of them
        vehicle.roadForce   = roadForce;   % N
        vehicle.Cd          = Cd;          % drag coeff
        vehicle.A           = A;           % frontal area, m2
        vehicle.weight      = weight;      % kg
        vehicle.payload     = payload;     % kg
        vehicle.overheadPwr = overheadPwr; % W
        vehicle.curbWeight  = weight + drivetrain.pack.weight;
        vehicle.maxWeight   = vehicle.curbWeight + payload;
        vehicle.rotWeight   = ((drivetrain.motor.inertia + drivetrain.gearInertia) * ...
                                drivetrain.gearRatio^2 + drivetrain.wheel.inertia*wheels)/...
                                drivetrain.wheel.radius^2;
        vehicle.equivMass   = vehicle.maxWeight + vehicle.rotWeight;
        vehicle.maxSpeed    = 2 * pi * drivetrain.wheel.radius * ...
        drivetrain.motor.RPMmax * 60 / (1000*drivetrain.gearRatio); % km/h
    end


end

