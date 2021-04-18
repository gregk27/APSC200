function [allData, scenario, sensors] = scenario_i()
%scenario_i - Returns sensor detections
%    allData = scenario_i returns sensor detections in a structure
%    with time for an internally defined scenario and sensor suite.
%
%    [allData, scenario, sensors] = scenario_i optionally returns
%    the drivingScenario and detection generator objects.

% Generated by MATLAB(R) 9.9 (R2020b) and Automated Driving Toolbox 3.2 (R2020b).
% Generated on: 01-Apr-2021 23:02:40

% Create the drivingScenario object and ego car
[scenario, egoVehicle] = createDrivingScenario;

% Create all the sensors
[sensors, numSensors] = createSensors(scenario);

scenario.StopTime = 20;

allData = struct('Time', {}, 'ActorPoses', {}, 'ObjectDetections', {}, 'PointClouds', {});
running = true;
while running
    
    % Generate the target poses of all actors relative to the ego vehicle
    poses = targetPoses(egoVehicle);
    time  = scenario.SimulationTime;
    
    objectDetections = {};
    laneDetections   = [];
    ptClouds = {};
    isValidTime = false(1, numSensors);
    isValidLaneTime = false(1, numSensors);
    isValidPointCloudTime = false(1, numSensors);
    
    % Generate detections for each sensor
    for sensorIndex = 1:numSensors
        sensor = sensors{sensorIndex};
        % Generate the ego vehicle lane boundaries
        type = getDetectorOutput(sensor);
        if strcmp(type, 'Objects only')
            [objectDets, numObjects, isValidTime(sensorIndex)] = sensor(poses, time);
            objectDetections = [objectDetections; objectDets(1:numObjects)]; %#ok<AGROW>
        elseif strcmp(type, 'Lanes and objects')
            [objectDets, numObjects, isValidTime(sensorIndex), laneDets, ~, isValidLaneTime(sensorIndex)] = sensor(poses, lanes, time);
            objectDetections = [objectDetections; objectDets(1:numObjects)]; %#ok<AGROW>
        elseif strcmp(type, 'PointCloud')
            if sensor.HasRoadsInputPort
                rdmesh = roadMesh(egoVehicle,min(500,sensor.MaxRange));
                [ptCloud, isValidPointCloudTime(sensorIndex)] = sensor(poses, rdmesh, time);
            else
                [ptCloud, isValidPointCloudTime(sensorIndex)] = sensor(poses, time);
            end
            ptClouds = [ptClouds; ptCloud]; %#ok<AGROW>
        end
    end
    
    % Aggregate all detections into a structure for later use
    if any(isValidTime) || any(isValidLaneTime) || any(isValidPointCloudTime)
        allData(end + 1) = struct( ...
            'Time',       scenario.SimulationTime, ...
            'ActorPoses', actorPoses(scenario), ...
            'ObjectDetections', {objectDetections}, ...
            'PointClouds',   {ptClouds}); %#ok<AGROW>
        process(scenario, objectDetections, ptClouds, egoVehicle, 'ivy');
    end
    
    % Advance the scenario one time step and exit the loop if the scenario is complete
    running = advance(scenario);
end

% Restart the driving scenario to return the actors to their initial positions.
restart(scenario);

% Release all the sensor objects so they can be used again.
for sensorIndex = 1:numSensors
    release(sensors{sensorIndex});
end

%%%%%%%%%%%%%%%%%%%%
% Helper functions %
%%%%%%%%%%%%%%%%%%%%

% Units used in createSensors and createDrivingScenario
% Distance/Position - meters
% Speed             - meters/second
% Angles            - degrees
% RCS Pattern       - dBsm

function [sensors, numSensors] = createSensors(scenario)
% createSensors Returns all sensor objects to generate detections

% Assign into each sensor the physical and radar profiles for all actors
profiles = actorProfiles(scenario);
sensors{1} = lidarPointCloudGenerator('SensorIndex', 1, ...
    'SensorLocation', [0.95 0], ...
    'ActorProfiles', profiles);
sensors{2} = visionDetectionGenerator('SensorIndex', 2, ...
    'SensorLocation', [1.5 0], ...
    'Yaw', -30, ...
    'MaxRange', 120, ...
    'DetectorOutput', 'Objects only', ...
    'Intrinsics', cameraIntrinsics([300 300],[320 240],[480 640]), ...
    'ActorProfiles', profiles);
numSensors = 2;

function [scenario, egoVehicle] = createDrivingScenario
% createDrivingScenario Returns the drivingScenario defined in the Designer

% Construct a drivingScenario object.
scenario = drivingScenario;

% Add all road segments
roadCenters = [30.7 5.9 0;
    33.5 17.1 0;
    40.1 17.7 0;
    42.6 -7.5 0;
    33.2 -6 0;
    30.7 5.9 0;
    16.4 15.6 0;
    4 3.4 0;
    17.2 -21 0;
    29.8 -18.3 0;
    30.7 5.9 0];
marking = [laneMarking('Solid', 'Color', [0.98 0.86 0.36])
    laneMarking('Solid', 'Color', [0.98 0.86 0.36])
    laneMarking('Solid')];
laneSpecification = lanespec(2, 'Marking', marking);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road');

% Add the ego vehicle
egoVehicle = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [47.7535649210999 9.53115737673791 0.01], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car');
waypoints = [47.7535649210999 9.53115737673791 0.01;
    48.68 5.64 0.01;
    48.5 0.2 0;
    45 -4.9 0;
    38.1 -6.7 0;
    33.92 -1.21 0.01;
    30.95 8.38 0.01;
    22.89 16.97 0.01;
    13 16.67 0.01;
    7.22 13.17 0.01;
    3.5 8.15 0.01;
    3.19 -10.33 0.01;
    13.23 -21.59 0.01;
    22.74 -23.87 0.01;
    31.64 -19.16 0.01;
    34.53 -10.49 0.01;
    32.2 9.8 0;
    34.5 15.9 0;
    40.5 16.3 0;
    45.3 12.8 0;
    47.92 8.38 0.01];
speed = [0;11;11;11;11;11;11;11;11;11;11;11;11;11;11;11;11;11;11;11;11];
waittime = [2;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0];
trajectory(egoVehicle, waypoints, speed, waittime);

% Add the non-ego actors
car1 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [38.1 19.9 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car1');
waypoints = [38.1 19.9 0;
    31.4 18.2 0;
    28.44 8.23 0.01;
    31.87 -9.34 0.01;
    29.36 -16.42 0.01;
    22.21 -19.92 0.01;
    14.83 -18.47 0.01;
    8.52 -13.45 0.01;
    6.54 -9.12 0.01;
    5.32 -1.81 0.01;
    7.22 6.4 0.01;
    9.58 10.28 0.01;
    13.84 12.64 0.01;
    22.05 13.25 0.01;
    29.9 0.9 0;
    37 -10.4 0;
    42.2 -9.8 0;
    48.5 -6.2 0;
    52.5 1.5 0;
    50.5 12 0;
    46.2 16.3 0;
    38.9 20 0];
speed = [11;11;11;11;11;11;11;11;11;11;11;11;11;11;11;11;11;11;11;11;11;11];
trajectory(car1, waypoints, speed);

pedestrian = actor(scenario, ...
    'ClassID', 4, ...
    'Length', 0.24, ...
    'Width', 0.45, ...
    'Height', 1.7, ...
    'Position', [38.4 11.1 0], ...
    'RCSPattern', [-8 -8;-8 -8], ...
    'Mesh', driving.scenario.pedestrianMesh, ...
    'Name', 'Pedestrian');
waypoints = [38.4 11.1 0;
    29.2 23.7 0];
speed = [1.5;1.5];
trajectory(pedestrian, waypoints, speed);

pedestrian1 = actor(scenario, ...
    'ClassID', 4, ...
    'Length', 0.24, ...
    'Width', 0.45, ...
    'Height', 1.7, ...
    'Position', [14.7 -4.3 0], ...
    'RCSPattern', [-8 -8;-8 -8], ...
    'Mesh', driving.scenario.pedestrianMesh, ...
    'Name', 'Pedestrian1');
waypoints = [14.7 -4.3 0;
    14.6 -4.1 0;
    -8.6 9.9 0];
speed = [1.5;1.5;1.5];
trajectory(pedestrian1, waypoints, speed);

bicycle = actor(scenario, ...
    'ClassID', 3, ...
    'Length', 1.7, ...
    'Width', 0.45, ...
    'Height', 1.7, ...
    'Position', [46.6 -13.6 0], ...
    'Mesh', driving.scenario.bicycleMesh, ...
    'Name', 'Bicycle');
waypoints = [46.6 -13.6 0;
    46.7 -13.4 0;
    39.4 -0.2 0];
speed = [5;5;5];
trajectory(bicycle, waypoints, speed);

vehicle(scenario, ...
    'ClassID', 7, ...
    'Length', 10, ...
    'Width', 5, ...
    'Position', [0.6 20.8 0], ...
    'Yaw', 20, ...
    'PlotColor', [183 70 255] / 255, ...
    'Name', 'User defined');

vehicle(scenario, ...
    'ClassID', 8, ...
    'Length', 13, ...
    'Width', 5, ...
    'Height', 5, ...
    'Position', [36.6 -23.1 0], ...
    'PlotColor', [0.301 0.745 0.933], ...
    'Name', 'User defined1');

vehicle(scenario, ...
    'ClassID', 9, ...
    'Length', 10, ...
    'Width', 10, ...
    'Position', [20 -5.2 0], ...
    'Yaw', -66, ...
    'PlotColor', [255 255 17] / 255, ...
    'Name', 'User defined2');

function output = getDetectorOutput(sensor)

if isa(sensor, 'visionDetectionGenerator')
    output = sensor.DetectorOutput;
elseif isa(sensor, 'lidarPointCloudGenerator')
    output = 'PointCloud';
else
    output = 'Objects only';
end
