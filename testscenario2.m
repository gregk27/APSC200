function [allData, scenario, sensors] = testscenario()
%testscenario - Returns sensor detections
%    allData = testscenario returns sensor detections in a structure
%    with time for an internally defined scenario and sensor suite.
%
%    [allData, scenario, sensors] = testscenario optionally returns
%    the drivingScenario and detection generator objects.

% Generated by MATLAB(R) 9.9 (R2020b) and Automated Driving Toolbox 3.2 (R2020b).
% Generated on: 22-Mar-2021 20:13:37

% Create the drivingScenario object and ego car
[scenario, egoVehicle] = createDrivingScenario;

% Create all the sensors
[sensors, numSensors] = createSensors(scenario);

scenario.StopTime = 20;
[textField, hTopViewAxes] = plotScenario(scenario, egoVehicle);

% User defined variables
objectArray = [];
delta = 8;

f1 = figure;
f2 = figure;

allData = struct('Time', {}, 'ActorPoses', {}, 'ObjectDetections', {}, 'LaneDetections', {}, 'PointClouds', {});
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
        maxLaneDetectionRange = min(500,sensor.MaxRange);
        lanes = laneBoundaries(egoVehicle, 'XDistance', linspace(-maxLaneDetectionRange, maxLaneDetectionRange, 101));
        type = getDetectorOutput(sensor);
        if strcmp(type, 'Objects only')
            [objectDets, numObjects, isValidTime(sensorIndex)] = sensor(poses, time);
            objectDetections = [objectDetections; objectDets(1:numObjects)]; %#ok<AGROW>
        elseif strcmp(type, 'Lanes only')
            [laneDets, ~, isValidTime(sensorIndex)] = sensor(lanes, time);
            laneDetections   = [laneDetections laneDets]; %#ok<AGROW>
        elseif strcmp(type, 'Lanes and objects')
            [objectDets, numObjects, isValidTime(sensorIndex), laneDets, ~, isValidLaneTime(sensorIndex)] = sensor(poses, lanes, time);
            objectDetections = [objectDetections; objectDets(1:numObjects)]; %#ok<AGROW>
            laneDetections   = [laneDetections laneDets]; %#ok<AGROW>
        elseif strcmp(type, 'Lanes with occlusion')
            [laneDets, ~, isValidLaneTime(sensorIndex)] = sensor(poses, lanes, time);
            laneDetections   = [laneDetections laneDets]; %#ok<AGROW>
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
            'LaneDetections', {laneDetections}, ...
            'PointClouds',   {ptClouds}); %#ok<AGROW>
    end
    
        
    if any(isValidTime) || any(isValidLaneTime) || any(isValidPointCloudTime)
       objects = objectDetections;
       objectsStruct = [objects{:}];
       
       allPosInertial = vehicle2Inertial(objects, egoVehicle);
       
       disp('Class ids: %i\n');
       disp([objectsStruct.ObjectClassID]);
       
       plot3(allPosInertial(1,:), allPosInertial(2,:), allPosInertial(3,:), 'b. ', 'Parent', hTopViewAxes);
       
       json = jsonencode(objectsStruct)
       inertial = jsonencode(allPosInertial)
       
       figure(f1);
       pcshow(ptClouds)
       figure(f2);
       [model1,inlierIndices,outlierIndices] = pcfitplane(ptCloud,1,[0,0,1],0.02);
        cloud = select(ptCloud,inlierIndices);
       [labels, numClusters] = pcsegdist(cloud, 0.5);
       pcshow(cloud.Location, labels);
       title('Point Cloud Clusters');
       
       message = sprintf('Number of objects sampled in one time step: %i\n', length(objects));
       textField.String = message;
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
sensors{1} = visionDetectionGenerator('SensorIndex', 1, ...
    'SensorLocation', [2.8 -0.9], ...
    'Yaw', -50, ...
    'Pitch', 0, ...
    'MaxRange', 15, ...
    'MinObjectImageSize', [0.1 1], ...
    'FalsePositivesPerImage', 0, ...
    'DetectorOutput', 'Objects only', ...
    'Intrinsics', cameraIntrinsics([100 99.9999999999999],[320 240],[480 640]), ...
    'ActorProfiles', profiles);
sensors{2} = lidarPointCloudGenerator('SensorIndex', 2, ...
    'SensorLocation', [0.95 0], ...
    'ActorProfiles', profiles);
numSensors = 2;

function [scenario, egoVehicle] = createDrivingScenario
% createDrivingScenario Returns the drivingScenario defined in the Designer

% Construct a drivingScenario object.
scenario = drivingScenario;

% Add all road segments
roadCenters = [40 40 0;
    40 15 0;
    -40 -15 0;
    -40 -40 0;
    0 -40 0;
    0 40 0;
    40 40 0];
marking = [laneMarking('Solid', 'Color', [0.98 0.86 0.36])
    laneMarking('Dashed')
    laneMarking('Dashed')];
laneSpecification = lanespec(2, 'Marking', marking);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road');

% Add the ego vehicle
egoVehicle = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [3.06830435408481 -2.0377180437017 0.01], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car');
waypoints = [3.06830435408481 -2.0377180437017 0.01;
    41.44 13.93 0.01;
    41.59 41.06 0.01;
    -1.56 41.11 0.01;
    -3.11 4.4 0.01;
    -0.43 -5.39 0.01;
    -1.27 -39.12 0.01;
    -38.44 -38.98 0.01;
    -38.63 -15.72 0.01;
    -4.85 -2.07 0.01;
    2.14 -2 0.01];
speed = [30;30;30;30;30;30;30;30;30;30;30];
trajectory(egoVehicle, waypoints, speed);

% Add the non-ego actors
truck = vehicle(scenario, ...
    'ClassID', 2, ...
    'Length', 8.2, ...
    'Width', 2.5, ...
    'Height', 3.5, ...
    'Position', [3.3 1.3 0], ...
    'Mesh', driving.scenario.truckMesh, ...
    'Name', 'Truck');
waypoints = [3.3 1.3 0;
    -41.94 -13.81 0.01;
    -41.9 -41.1 0;
    2.2 -40.4 0;
    3.3 -3.9 0;
    0.4 5.9 0;
    1.6 38.3 0;
    38.6 39.4 0;
    38.8 16.3 0;
    4.6 2.4 0];
speed = [40;40;40;40;40;40;40;40;40;40];
trajectory(truck, waypoints, speed);

vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [18.5 -2.1 0], ...
    'Yaw', 13, ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car1');

vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [23.7333333333333 -0.77 0], ...
    'Yaw', 21, ...
    'Mesh', driving.scenario.carMesh, ...
    'PlotColor', [0.929 0.694 0.125], ...
    'Name', 'Car2');

vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [28.6766666666667 1.26 0], ...
    'Yaw', 28, ...
    'Mesh', driving.scenario.carMesh, ...
    'PlotColor', [0.929 0.694 0.125], ...
    'Name', 'Car3');

function output = getDetectorOutput(sensor)

if isa(sensor, 'visionDetectionGenerator')
    output = sensor.DetectorOutput;
elseif isa(sensor, 'lidarPointCloudGenerator')
    output = 'PointCloud';
else
    output = 'Objects only';
end

