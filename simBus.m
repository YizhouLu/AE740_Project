% function results = simVehicle(vehicle,cycle,grade)
%
%    Runs a simulation of the electric vehicle.
%
%    Inputs:
%      vehicle: a structure that is set up by setupSimVehicle.m and defines 
%               the characteristics of the vehicle to be simulated
%      speed:   a 1x2 matrix, where first column is time in seconds and 
%               second column is desired speed in miles per hour. Since we
%               are running real time, the velocity input in the second
%               column is the present desired vehicle velocity so the time
%               associated will always be 1 for each iteration
%    Outputs:
%      results: a data structure containing all simulation intermediate
%               results as fields (see code for all fields of the
%               "results.xxx" type, and see text for description).

% Need to save prevSpeed and prevMotorSpeed so that we can use them for the
% proceeding iteration to solve the battery current profile. Save
% prevDistance and use it as an input to get how far the bus has driven.

function results = simBus(vehicle, speed, prevSpeed, prevMotorSpeed, prevDistance)
    
    rho = 1.225; % air density, kg/m3

    % structure containing all bus specs which match Proterra's 35 foot
    % Catalyst XR bus with ProDrive Drivetrain
    results.vehicle = vehicle;
    
    % we're not going to mess around with grade
    results.grade = 0; 
  
    results.desSpeedKPH = speed * 1.609344; % [miles/hr] -> [km/hour]
    results.desSpeed    = min(vehicle.maxSpeed/3.6,results.desSpeedKPH/3.6); % m/s
    
    delta_speed = 1; % always incrementing by 1 second in the simulation

    % results.desAccel       [m/s2]
    % results.desAccelForce     [N]
    % results.aeroForce         [N]
    % results.rollGradeForce    [N]
    % results.demandTorque    [N-m]
    % results.maxTorque       [N-m]
    % results.limitRegen      [N-m]
    % results.limitTorque     [N-m]
    % results.motorTorque     [N-m]
    % results.demandPower      [kW]
    % results.limitPower       [kW]
    % results.batteryDemand    [kW]
    % results.current           [A] 
    % results.batterySOC         []
    % results.actualAccelForce  [N]
    % results.actualAccel    [m/s2]
    % results.motorSpeed      [RPM]
    % results.actualSpeed     [m/s]
    % results.actualSpeedKPH [km/h]
    % results.distance         [km]
  
    %% Desired values
    % calculate the desired acceleration
    results.desAccel = (results.desSpeed - prevSpeed)/ delta_speed;
    
    % Newton's law to compute desired force produced at the roads surface.
    % Force = ma where a from step above and mult by vehicle mass. Need the
    % equivalent mass to account for stationary + rotating mass
    results.desAccelForce = vehicle.equivMass * results.desAccel;
    
    % drag and rollGrade acts as negative forces in sum of forces for F =
    % ma.
    results.aeroForce = 0.5*rho * vehicle.Cd * vehicle.A * prevSpeed^2;
    results.rollGradeForce = vehicle.maxWeight * 9.81 * sin(results.grade); % grade = 0
    
    if abs(prevSpeed) > 0
        results.rollGradeForce = results.rollGradeForce + ...
        vehicle.drivetrain.wheel.rollCoef * vehicle.maxWeight * 9.81;
    end
    
    % the torque we need from the motor to compute the desired acceleration
    % based on the velocity profile we are simulating
    results.demandTorque = (results.desAccelForce + results.aeroForce + ...
        results.rollGradeForce + vehicle.roadForce) * ...
        vehicle.drivetrain.wheel.radius / vehicle.drivetrain.gearRatio;

    % check to see which regime of the motor we are operating in based on
    % the previous value of motor speed to get the torque we are allowed to
    % supply.
    if prevMotorSpeed < vehicle.drivetrain.motor.RPMrated
        results.maxTorque = vehicle.drivetrain.motor.Lmax;
        % riding the constant torque portion of the Torque [Nm] vs Speed
        % [RPM] curve. Lmax = 900 Nm. 
    else
        % if the motor speed is above the rated RPM value of 2654 RPM, then
        % the max torque is determined by the fraction of
        % RPMrated/actualMotorSpeed * Lmax which will always be less than
        % 900 Nm. This is the constant power portion of the curve. Power =
        % torque x motor_speed. Motor speed must always be less than max
        % RPM which is 5500 RPM for this vehicle.
        results.maxTorque = vehicle.drivetrain.motor.Lmax * ...
        vehicle.drivetrain.motor.RPMrated / prevMotorSpeed;
    end
    
    % Consider when negative torque is demanded and a regen event occurs.
    % The most we can regen is the minimum between the max torque * the
    % fractional regen torque limit or just the else portion of the
    % previous if-else statement. The first argumnet is the if part, the
    % second argument is the else part.
    results.limitRegen = min(vehicle.drivetrain.regenTorque * vehicle.drivetrain.motor.Lmax,...
        results.maxTorque);
    
    % Now we figure out whether we need to limit the demanded torque which
    % satisfies the velocity profile. If that torque is small, then it's no
    % problem and we take it, otherwise use the maxTorque just calculated
    results.limitTorque = min(results.demandTorque, results.maxTorque);
    
    % Now enforce the limitRegen piece we just wrote on line 107. This only
    % factors in if the vehicle is decelerating. Take the larger value of
    % either the limited Regen torque or the limited torque.
    if results.limitTorque > 0
        results.motorTorque = results.limitTorque;
    else
        results.motorTorque = max(-results.limitRegen, results.limitTorque);
    end

    %% Actual Values
    % Finally, we have an actual acceleration force we can produce based on
    % the actual torque that we can dish out.
    results.actualAccelForce = results.limitTorque * vehicle.drivetrain.gearRatio / ...
        vehicle.drivetrain.wheel.radius - results.aeroForce - results.rollGradeForce - ...
        vehicle.roadForce;
    
    % The actual acceleration based on the actual acceleration force we can
    % dish out
    results.actualAccel = results.actualAccelForce / vehicle.equivMass;
    
    % The motor speed resulting from this actual acceleration. This actual
    % acceleration might cause the motor to spin at a higher angular
    % velocity than its max RPM. So compute an RPM first then limit the RPM
    % just in case it is above max speed
    results.motorSpeed = min(vehicle.drivetrain.motor.RPMmax,...
        vehicle.drivetrain.gearRatio * (prevSpeed + results.actualAccel * ...
        delta_speed) * 60 / (2*pi*vehicle.drivetrain.wheel.radius));
    
    % the actual speed resulting from the actual motor speed
    results.actualSpeed = results.motorSpeed * 2*pi*vehicle.drivetrain.wheel.radius / ...
        (60 * vehicle.drivetrain.gearRatio); % m/s
    
    % convert this to kilometers/hour
    results.actualSpeedKPH = results.actualSpeed * 3600/1000;
    
    % compute how far you have moved
    deltadistance = (results.actualSpeed + prevSpeed)/2 * delta_speed/1000;
    results.distance = prevDistance + deltadistance;

    %% Battery Power Calculations
    % Compute the power demanded by the motor. This if-else statement just
    % considers either positive demanded power (dishcarge) or negative
    % demanded power (regen)
    if results.limitTorque > 0
        results.demandPower = results.limitTorque;
    else
        results.demandPower = max(results.limitTorque, -results.limitRegen);
    end
    
    % Convert to kW
    results.demandPower = results.demandPower * 2*pi * (prevMotorSpeed + results.motorSpeed)/2/60000;
    
    
    results.limitPower = max(-vehicle.drivetrain.motor.maxPower, min(...
        vehicle.drivetrain.motor.maxPower,results.demandPower));
    
    results.batteryDemand = vehicle.overheadPwr/1000;
    
    if results.limitPower > 0
        results.batteryDemand = results.batteryDemand + ...
            results.limitPower/vehicle.drivetrain.efficiency;
    else
        results.batteryDemand = results.batteryDemand + ...
            results.limitPower*vehicle.drivetrain.efficiency;
    end
    
    % we should output the power and solve for the current profile based on
    % the simulated pack voltage and this derived power demand. When were
    % really feeling ourselves, we should enforce power limits here and
    % then use that to compute battery pack current.
    
    results.batteryDemand = results.batteryDemand*1000; % Watts
    
%     results.current = results.batteryDemand*1000/vehicle.drivetrain.pack.vnom;
    
    % now we have current which we will use to update our pack SOC, voltage
    % and cell temperatures in the main script.
    
  end