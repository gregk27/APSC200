function [allData, scenario, sensor] = City()
%City - Returns sensor detections
%    allData = City returns sensor detections in a structure
%    with time for an internally defined scenario and sensor suite.
%
%    [allData, scenario, sensors] = City optionally returns
%    the drivingScenario and detection generator objects.

% Generated by MATLAB(R) 9.9 (R2020b) and Automated Driving Toolbox 3.2 (R2020b).
% Generated on: 30-Mar-2021 23:10:44

% Create the drivingScenario object and ego car
[scenario, egoVehicle] = createDrivingScenario;

% Create all the sensors
sensor = createSensor(scenario);

allData = struct('Time', {}, 'ActorPoses', {}, 'ObjectDetections', {}, 'LaneDetections', {}, 'PointClouds', {});
running = true;
while running
    
    % Generate the target poses of all actors relative to the ego vehicle
    poses = targetPoses(egoVehicle);
    time  = scenario.SimulationTime;
    
    % Generate detections for the sensor
    laneDetections = [];
    objectDetections = [];
    if sensor.HasRoadsInputPort
        rdmesh = roadMesh(egoVehicle,min(500,sensor.MaxRange));
        [ptClouds, isValidPointCloudTime] = sensor(poses, rdmesh, time);
    else
        [ptClouds, isValidPointCloudTime] = sensor(poses, time);
    end
    
    % Aggregate all detections into a structure for later use
    if isValidPointCloudTime
        allData(end + 1) = struct( ...
            'Time',       scenario.SimulationTime, ...
            'ActorPoses', actorPoses(scenario), ...
            'ObjectDetections', {objectDetections}, ...
            'LaneDetections', {laneDetections}, ...
            'PointClouds',   {ptClouds}); %#ok<AGROW>
        process(scenario, objectDetections, ptClouds, egoVehicle)
    end
    
    % Advance the scenario one time step and exit the loop if the scenario is complete
    running = advance(scenario);
end

% Restart the driving scenario to return the actors to their initial positions.
restart(scenario);

% Release the sensor object so it can be used again.
release(sensor);

%%%%%%%%%%%%%%%%%%%%
% Helper functions %
%%%%%%%%%%%%%%%%%%%%

% Units used in createSensors and createDrivingScenario
% Distance/Position - meters
% Speed             - meters/second
% Angles            - degrees
% RCS Pattern       - dBsm

function sensor = createSensor(scenario)
% createSensors Returns all sensor objects to generate detections

% Assign into each sensor the physical and radar profiles for all actors
profiles = actorProfiles(scenario);
sensor = lidarPointCloudGenerator('SensorIndex', 1, ...
    'SensorLocation', [0.95 0], ...
    'ActorProfiles', profiles);

function [scenario, egoVehicle] = createDrivingScenario
% createDrivingScenario Returns the drivingScenario defined in the Designer

% Construct a drivingScenario object.
scenario = drivingScenario;

% Add all road segments
roadCenters = [37.2 40.4 0;
    60.3 40.4 0];
road(scenario, roadCenters, 'Name', 'Road');

roadCenters = [60 43.4 0;
    60 -30 0];
road(scenario, roadCenters, 'Name', 'Road1');

roadCenters = [40.1 21.8 0;
    40.1 40.2 0];
road(scenario, roadCenters, 'Name', 'Road2');

roadCenters = [57.2 20 0;
    -8.6 20 0];
road(scenario, roadCenters, 'Name', 'Road3');

roadCenters = [19.5 20.3 0;
    19.5 -20 0];
road(scenario, roadCenters, 'Name', 'Road4');

roadCenters = [-10 -20 0;
    63 -20 0];
road(scenario, roadCenters, 'Name', 'Road5');

roadCenters = [-10 -23 0;
    -10 23.1 0];
road(scenario, roadCenters, 'Name', 'Road6');

% Add the ego vehicle
egoVehicle = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [-5.44 18.63 0.01], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car');
waypoints = [-5.08 18.7 0.01;
    1.9 18.72 0.01;
    38.09 18.64 0.01;
    53.72 18.79 0.01;
    58.73 19.33 0.01;
    61.03 23.01 0.01;
    61.58 30.95 0.01;
    60.79 38.37 0.01;
    58.15 41.2 0.01;
    53.78 41.72 0.01;
    44.54 41.77 0.01;
    39.83 40.18 0.01;
    38.66 35.74 0.01;
    38.61 27.46 0.01;
    40.27 20.2 0.01;
    44.65 18.89 0.01;
    55.52 18.4 0.01;
    58.52 16.47 0.01;
    58.96 11.97 0;
    58.66 -1.93 0;
    59.29 -14.67 0.01;
    58.13 -18.48 0.01;
    50.06 -18.63 0;
    40.56 -18.33 0;
    32.66 -18.13 0;
    23.89 -18.48 0.01;
    21.68 -17.74 0.01;
    20.14 -12.67 0.01;
    20.24 1.53 0.01;
    20.4 11.77 0.01;
    20.26 16.67 0;
    16.92 19.86 0.01;
    10.1 20.55 0.01;
    -4.85 20.61 0.01;
    -9.55 18.72 0.01;
    -10.92 11.51 0.01;
    -10.78 -2.09 0.01;
    -10.78 -13.2 0.01;
    -10.14 -20.27 0.01;
    -4.7 -21.97 0.01;
    13.27 -21.47 0.01];
speed = [30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30];
trajectory(egoVehicle, waypoints, speed);

% Add the non-ego actors
actor(scenario, ...
    'ClassID', 6, ...
    'Length', 10, ...
    'Width', 10, ...
    'Height', 50, ...
    'Position', [50.1 10 0], ...
    'Name', 'Building');

vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [48.23 16.78 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car1');

vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [39.67 16.59 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car2');

actor(scenario, ...
    'ClassID', 6, ...
    'Length', 10, ...
    'Width', 10, ...
    'Height', 50, ...
    'Position', [30 10 0], ...
    'Name', 'Building1');

actor(scenario, ...
    'ClassID', 6, ...
    'Length', 10, ...
    'Width', 10, ...
    'Height', 50, ...
    'Position', [50.3 30.2 0], ...
    'Name', 'Building2');

actor(scenario, ...
    'ClassID', 6, ...
    'Length', 10, ...
    'Width', 10, ...
    'Height', 50, ...
    'Position', [30.1 30.1 0], ...
    'Name', 'Building3');

actor(scenario, ...
    'ClassID', 6, ...
    'Length', 10, ...
    'Width', 10, ...
    'Height', 50, ...
    'Position', [50.1 -9.8 0], ...
    'Name', 'Building4');

actor(scenario, ...
    'ClassID', 6, ...
    'Length', 10, ...
    'Width', 10, ...
    'Height', 50, ...
    'Position', [30.2 -9.6 0], ...
    'Name', 'Building5');

vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [22.6 4.6 0], ...
    'Yaw', 90, ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car3');

vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [22.6 -8.6 0], ...
    'Yaw', 90, ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car4');

vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [12.8 23.1 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car5');

vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [6.5 23.4 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car6');

vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [-1 23.1 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car7');

vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [-13.1 0.4 0], ...
    'Yaw', 90, ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car8');

actor(scenario, ...
    'ClassID', 6, ...
    'Length', 10, ...
    'Width', 10, ...
    'Height', 50, ...
    'Position', [5.2 10.3 0], ...
    'Name', 'Building6');

actor(scenario, ...
    'ClassID', 6, ...
    'Length', 10, ...
    'Width', 10, ...
    'Height', 50, ...
    'Position', [10.3 -9.8 0], ...
    'Name', 'Building7');

actor(scenario, ...
    'ClassID', 6, ...
    'Length', 10, ...
    'Width', 10, ...
    'Height', 50, ...
    'Position', [0 -9.7 0], ...
    'Name', 'Building8');
