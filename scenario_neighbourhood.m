function [allData, scenario, sensor] = neighbourhood()
%neighbourhood - Returns sensor detections
%    allData = neighbourhood returns sensor detections in a structure
%    with time for an internally defined scenario and sensor suite.
%
%    [allData, scenario, sensors] = neighbourhood optionally returns
%    the drivingScenario and detection generator objects.

% Generated by MATLAB(R) 9.9 (R2020b) and Automated Driving Toolbox 3.2 (R2020b).
% Generated on: 31-Mar-2021 00:35:00

% Create the drivingScenario object and ego car
[scenario, egoVehicle] = createDrivingScenario;

% Create all the sensors
sensor = createSensor(scenario);

scenario.StopTime = 200;

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
        process(scenario, objectDetections, ptClouds, egoVehicle);
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
roadCenters = [44.97 57.45 0;
    44.7 34.8 0;
    34.6 15 0;
    28.3 13.7 0;
    -12.9 7.6 0;
    -22.4 6.1 0;
    -40.5 4.8 0;
    -60.7 4.8 0];
road(scenario, roadCenters, 'Name', 'Road');

roadCenters = [-12.7 6.6 0;
    -12.5 -20.3 0];
road(scenario, roadCenters, 'Name', 'Road1');

roadCenters = [60.5 -20 0;
    -60 -20.1 0];
laneSpecification = lanespec([1 1]);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road2');

roadCenters = [60.5 -20 0;
    60.4 -54.6 0];
road(scenario, roadCenters, 'Name', 'Road3');

roadCenters = [-5.8 -52.8 0;
    -6.1 -102.1 0];
road(scenario, roadCenters, 'Name', 'Road4');

roadCenters = [-6.1 -102.1 0;
    -5.7 -140 0];
road(scenario, roadCenters, 'Name', 'Road5');

% Add the ego vehicle
egoVehicle = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [44.0564591969349 56.3294994386381 0.01], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car');
waypoints = [44.0564591969349 56.3294994386381 0.01;
    43.5 34.76 0.01;
    34.32 16.33 0.01;
    28.16 14.98 0.01;
    -8.34000000000001 9.57000000000001 0.01;
    -13 7.78 0.01;
    -13.91 2.81 0.01;
    -13.53 -14.79 0.01;
    -9.88000000000001 -21.18 0;
    -2.47000000000001 -22.32 0.01;
    15.21 -21.87 0.01;
    18.31 -22.69 0.01;
    19.51 -25.95 0.01;
    19.62 -52.28 0;
    20.22 -87.58 0;
    18.84 -96 0.01;
    15.22 -98.84 0.01;
    8.26999999999999 -100.43 0.01;
    -27.88 -101.05 0.01;
    -35.18 -101.18 0;
    -44.48 -98.58 0;
    -49.18 -86.58 0;
    -48.98 -75.08 0;
    -48.78 -62.12 0.01;
    -45.91 -53.12 0.01;
    -39.63 -50.4 0.01;
    -29.33 -51.13 0.01;
    -17.56 -52.84 0.01;
    10.92 -56.07 0.01;
    18.14 -55.51 0.01;
    20.69 -53.53 0.01;
    22.1 -47.01 0;
    22.4 -25.6 0;
    20.3 -18.6 0;
    17.4 -17.9 0;
    -2.78 -18 0.01;
    -28.66 -18.3 0.01;
    -58.09 -18.2 0.01];
speed = [30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;0;30;30;30;30;30;30;30;30;30;30;20;30;30;30;30;30;30;30;30;30];
waittime = [0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0.2;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0];
trajectory(egoVehicle, waypoints, speed, waittime);

% Add the non-ego actors
vehicle(scenario, ...
    'ClassID', 7, ...
    'Length', 40, ...
    'Width', 15, ...
    'Height', 12, ...
    'Position', [-59.1 -7.3 0], ...
    'Name', 'Apartment');

vehicle(scenario, ...
    'ClassID', 7, ...
    'Length', 40, ...
    'Width', 15, ...
    'Height', 12, ...
    'Position', [25.1666666666667 -7.1 0], ...
    'PlotColor', [0.85 0.325 0.098], ...
    'Name', 'Apartment1');

vehicle(scenario, ...
    'ClassID', 8, ...
    'Length', 15, ...
    'Width', 10, ...
    'Height', 3, ...
    'Position', [58 55.4 0], ...
    'Yaw', -91, ...
    'Name', 'House 1');

vehicle(scenario, ...
    'ClassID', 8, ...
    'Length', 15, ...
    'Width', 10, ...
    'Height', 3, ...
    'Position', [35.8 5.9 0], ...
    'Yaw', -168, ...
    'PlotColor', [0.494 0.184 0.556], ...
    'Name', 'House 2');

vehicle(scenario, ...
    'ClassID', 8, ...
    'Length', 15, ...
    'Width', 10, ...
    'Height', 3, ...
    'Position', [-43.3 15.9 0], ...
    'Yaw', 180, ...
    'PlotColor', [0.494 0.184 0.556], ...
    'Name', 'House 3');

vehicle(scenario, ...
    'ClassID', 8, ...
    'Length', 15, ...
    'Width', 10, ...
    'Height', 3, ...
    'Position', [15.4 -30.2 0], ...
    'Yaw', 180, ...
    'PlotColor', [0.494 0.184 0.556], ...
    'Name', 'House 4');

vehicle(scenario, ...
    'ClassID', 8, ...
    'Length', 15, ...
    'Width', 10, ...
    'Height', 3, ...
    'Position', [-12 -65.4 0], ...
    'Yaw', 180, ...
    'PlotColor', [0.494 0.184 0.556], ...
    'Name', 'House 5');

vehicle(scenario, ...
    'ClassID', 8, ...
    'Length', 15, ...
    'Width', 10, ...
    'Height', 3, ...
    'Position', [12.2 -88.6 0], ...
    'Yaw', 180, ...
    'PlotColor', [0.494 0.184 0.556], ...
    'Name', 'House 6');

vehicle(scenario, ...
    'ClassID', 8, ...
    'Length', 15, ...
    'Width', 10, ...
    'Height', 3, ...
    'Position', [31 -54.9 0], ...
    'Yaw', -90, ...
    'PlotColor', [0.494 0.184 0.556], ...
    'Name', 'House 7');

vehicle(scenario, ...
    'ClassID', 8, ...
    'Length', 15, ...
    'Width', 10, ...
    'Height', 3, ...
    'Position', [-24.5 -112.5 0], ...
    'Yaw', 180, ...
    'Name', 'House 8');

vehicle(scenario, ...
    'ClassID', 9, ...
    'Length', 20, ...
    'Width', 10, ...
    'Height', 3, ...
    'Position', [53.9 21.5 0], ...
    'Yaw', -127, ...
    'Name', 'House 9');

vehicle(scenario, ...
    'ClassID', 9, ...
    'Length', 20, ...
    'Width', 10, ...
    'Height', 3, ...
    'Position', [-21.3666666666667 16.7 0], ...
    'Yaw', -177, ...
    'PlotColor', [0.466 0.674 0.188], ...
    'Name', 'House 10');

vehicle(scenario, ...
    'ClassID', 9, ...
    'Length', 20, ...
    'Width', 10, ...
    'Height', 3, ...
    'Position', [31.3 -28.8 0], ...
    'Yaw', -90, ...
    'PlotColor', [0.466 0.674 0.188], ...
    'Name', 'House 11');

vehicle(scenario, ...
    'ClassID', 9, ...
    'Length', 20, ...
    'Width', 10, ...
    'Height', 3, ...
    'Position', [-43.1666666666667 -36.9 0], ...
    'Yaw', -147, ...
    'PlotColor', [0.466 0.674 0.188], ...
    'Name', 'House 12');

vehicle(scenario, ...
    'ClassID', 9, ...
    'Length', 20, ...
    'Width', 10, ...
    'Height', 3, ...
    'Position', [-60.5333333333333 -99.2 0], ...
    'Yaw', -51, ...
    'PlotColor', [0.466 0.674 0.188], ...
    'Name', 'House 13');

vehicle(scenario, ...
    'ClassID', 9, ...
    'Length', 20, ...
    'Width', 10, ...
    'Height', 3, ...
    'Position', [-15.1 -75.9 0], ...
    'Yaw', -90, ...
    'PlotColor', [0.466 0.674 0.188], ...
    'Name', 'House 14');

vehicle(scenario, ...
    'ClassID', 10, ...
    'Length', 10, ...
    'Width', 10, ...
    'Height', 5, ...
    'Position', [52.9 32.1 0], ...
    'Yaw', -13, ...
    'Name', 'House 15');

vehicle(scenario, ...
    'ClassID', 10, ...
    'Length', 10, ...
    'Width', 10, ...
    'Height', 5, ...
    'Position', [9.46666666666665 0.600000000000001 0], ...
    'Yaw', 8, ...
    'PlotColor', [0.494 0.184 0.556], ...
    'Name', 'House 16');

vehicle(scenario, ...
    'ClassID', 10, ...
    'Length', 10, ...
    'Width', 10, ...
    'Height', 5, ...
    'Position', [3.53333333333332 -66.7 0], ...
    'Yaw', -1, ...
    'PlotColor', [0.494 0.184 0.556], ...
    'Name', 'House 17');

vehicle(scenario, ...
    'ClassID', 10, ...
    'Length', 10, ...
    'Width', 10, ...
    'Height', 5, ...
    'Position', [-40.4 -65.3 0], ...
    'Yaw', 1, ...
    'PlotColor', [0.494 0.184 0.556], ...
    'Name', 'House 18');

vehicle(scenario, ...
    'ClassID', 8, ...
    'Length', 15, ...
    'Width', 10, ...
    'Height', 3, ...
    'Position', [-19.7 -38.5 0], ...
    'Yaw', 174, ...
    'PlotColor', [0.494 0.184 0.556], ...
    'Name', 'House 19');

vehicle(scenario, ...
    'ClassID', 8, ...
    'Length', 15, ...
    'Width', 10, ...
    'Height', 3, ...
    'Position', [-0.400000000000005 -41.3 0], ...
    'Yaw', 173, ...
    'PlotColor', [0.494 0.184 0.556], ...
    'Name', 'House 20');

vehicle(scenario, ...
    'ClassID', 9, ...
    'Length', 20, ...
    'Width', 10, ...
    'Height', 3, ...
    'Position', [-2.13333333333333 -29.7 0], ...
    'Yaw', 179, ...
    'PlotColor', [0.466 0.674 0.188], ...
    'Name', 'House 21');

vehicle(scenario, ...
    'ClassID', 10, ...
    'Length', 10, ...
    'Width', 10, ...
    'Height', 5, ...
    'Position', [6.46666666666665 -43.2 0], ...
    'Yaw', 1, ...
    'Name', 'House 22');

vehicle(scenario, ...
    'ClassID', 8, ...
    'Length', 15, ...
    'Width', 10, ...
    'Height', 3, ...
    'Position', [-61 -56.7 0], ...
    'Yaw', -91, ...
    'PlotColor', [0.494 0.184 0.556], ...
    'Name', 'House 23');

vehicle(scenario, ...
    'ClassID', 8, ...
    'Length', 15, ...
    'Width', 10, ...
    'Height', 3, ...
    'Position', [-61 -77.3 0], ...
    'Yaw', -91, ...
    'PlotColor', [0.494 0.184 0.556], ...
    'Name', 'House 24');

vehicle(scenario, ...
    'ClassID', 8, ...
    'Length', 15, ...
    'Width', 10, ...
    'Height', 3, ...
    'Position', [-2.5 19.7 0], ...
    'Yaw', -169, ...
    'PlotColor', [0.494 0.184 0.556], ...
    'Name', 'House 25');

vehicle(scenario, ...
    'ClassID', 8, ...
    'Length', 15, ...
    'Width', 10, ...
    'Height', 3, ...
    'Position', [14.8 21.7 0], ...
    'Yaw', -175, ...
    'PlotColor', [0.494 0.184 0.556], ...
    'Name', 'House 26');

vehicle(scenario, ...
    'ClassID', 9, ...
    'Length', 20, ...
    'Width', 10, ...
    'Height', 3, ...
    'Position', [33.7666666666667 36.3 0], ...
    'Yaw', -134, ...
    'PlotColor', [0.466 0.674 0.188], ...
    'Name', 'House 27');

vehicle(scenario, ...
    'ClassID', 8, ...
    'Length', 15, ...
    'Width', 10, ...
    'Height', 3, ...
    'Position', [-1.5 0.999999999999996 0], ...
    'Yaw', -90, ...
    'PlotColor', [0.494 0.184 0.556], ...
    'Name', 'House 28');

vehicle(scenario, ...
    'ClassID', 8, ...
    'Length', 15, ...
    'Width', 10, ...
    'Height', 3, ...
    'Position', [34.6 55 0], ...
    'Yaw', -90, ...
    'Name', 'House 29');

vehicle(scenario, ...
    'ClassID', 9, ...
    'Length', 20, ...
    'Width', 10, ...
    'Height', 3, ...
    'Position', [17.7333333333333 -112.9 0], ...
    'Yaw', 180, ...
    'Name', 'House 30');

vehicle(scenario, ...
    'ClassID', 10, ...
    'Length', 10, ...
    'Width', 10, ...
    'Height', 5, ...
    'Position', [-19.4333333333334 -112.8 0], ...
    'Yaw', 1, ...
    'PlotColor', [0.494 0.184 0.556], ...
    'Name', 'House 31');

vehicle(scenario, ...
    'ClassID', 9, ...
    'Length', 20, ...
    'Width', 10, ...
    'Height', 3, ...
    'Position', [31.2333333333333 -77.5 0], ...
    'Yaw', -90, ...
    'PlotColor', [0.466 0.674 0.188], ...
    'Name', 'House 32');

vehicle(scenario, ...
    'ClassID', 10, ...
    'Length', 10, ...
    'Width', 10, ...
    'Height', 5, ...
    'Position', [23.8333333333333 -104 0], ...
    'Yaw', -41, ...
    'PlotColor', [0.494 0.184 0.556], ...
    'Name', 'House 33');

vehicle(scenario, ...
    'ClassID', 11, ...
    'Length', 1, ...
    'Width', 1, ...
    'Height', 7, ...
    'Position', [-41.05 -89.82 0], ...
    'RearOverhang', 0, ...
    'FrontOverhang', 0, ...
    'Name', 'Tree');

vehicle(scenario, ...
    'ClassID', 11, ...
    'Length', 1, ...
    'Width', 1, ...
    'Height', 7, ...
    'Position', [-43.7233333333333 -79.22 0], ...
    'RearOverhang', 0, ...
    'FrontOverhang', 0, ...
    'PlotColor', [0.85 0.325 0.098], ...
    'Name', 'Tree1');

vehicle(scenario, ...
    'ClassID', 11, ...
    'Length', 1, ...
    'Width', 1, ...
    'Height', 7, ...
    'Position', [-30.6266666666667 -89.73 0], ...
    'RearOverhang', 0, ...
    'FrontOverhang', 0, ...
    'PlotColor', [0.85 0.325 0.098], ...
    'Name', 'Tree2');

vehicle(scenario, ...
    'ClassID', 11, ...
    'Length', 1, ...
    'Width', 1, ...
    'Height', 7, ...
    'Position', [-30.01 -95.87 0], ...
    'RearOverhang', 0, ...
    'FrontOverhang', 0, ...
    'PlotColor', [0.85 0.325 0.098], ...
    'Name', 'Tree3');

vehicle(scenario, ...
    'ClassID', 11, ...
    'Length', 1, ...
    'Width', 1, ...
    'Height', 7, ...
    'Position', [-34.4433333333333 -94.25 0], ...
    'RearOverhang', 0, ...
    'FrontOverhang', 0, ...
    'PlotColor', [0.85 0.325 0.098], ...
    'Name', 'Tree4');

vehicle(scenario, ...
    'ClassID', 11, ...
    'Length', 1, ...
    'Width', 1, ...
    'Height', 7, ...
    'Position', [-42.7166666666667 -85.15 0], ...
    'RearOverhang', 0, ...
    'FrontOverhang', 0, ...
    'PlotColor', [0.85 0.325 0.098], ...
    'Name', 'Tree5');

vehicle(scenario, ...
    'ClassID', 11, ...
    'Length', 1, ...
    'Width', 1, ...
    'Height', 7, ...
    'Position', [-37.99 -93.11 0], ...
    'RearOverhang', 0, ...
    'FrontOverhang', 0, ...
    'PlotColor', [0.85 0.325 0.098], ...
    'Name', 'Tree6');

vehicle(scenario, ...
    'ClassID', 11, ...
    'Length', 1, ...
    'Width', 1, ...
    'Height', 7, ...
    'Position', [-33.6933333333333 -88.71 0], ...
    'RearOverhang', 0, ...
    'FrontOverhang', 0, ...
    'Name', 'Tree7');

vehicle(scenario, ...
    'ClassID', 11, ...
    'Length', 1, ...
    'Width', 1, ...
    'Height', 7, ...
    'Position', [-37.6966666666667 -80.29 0], ...
    'RearOverhang', 0, ...
    'FrontOverhang', 0, ...
    'PlotColor', [0.85 0.325 0.098], ...
    'Name', 'Tree8');

vehicle(scenario, ...
    'ClassID', 11, ...
    'Length', 1, ...
    'Width', 1, ...
    'Height', 7, ...
    'Position', [-36.63 -86.08 0], ...
    'RearOverhang', 0, ...
    'FrontOverhang', 0, ...
    'PlotColor', [0.85 0.325 0.098], ...
    'Name', 'Tree9');

truck = vehicle(scenario, ...
    'ClassID', 2, ...
    'Length', 8.2, ...
    'Width', 2.5, ...
    'Height', 3.5, ...
    'Position', [-77.8837404743875 -21.8881345490185 0], ...
    'EntryTime', 10, ...
    'Mesh', driving.scenario.truckMesh, ...
    'Name', 'Truck');
waypoints = [-77.8837404743875 -21.8881345490185 0;
    -56.61 -21.83 0.01;
    -13.63 -21.91 0.01;
    58.65 -22.19 0.01;
    60.5 -21.8 0.01;
    61.26 -20.16 0;
    60.5 -18.18 0;
    58.52 -17.8 0.01;
    -57.13 -18.18 0.01];
speed = [30;30;30;30;30;30;30;30;30];
trajectory(truck, waypoints, speed);

truck1 = vehicle(scenario, ...
    'ClassID', 2, ...
    'Length', 8.2, ...
    'Width', 2.5, ...
    'Height', 3.5, ...
    'Position', [-74.4551715575164 -21.7846879943097 0], ...
    'EntryTime', 8, ...
    'Mesh', driving.scenario.truckMesh, ...
    'PlotColor', [0.466 0.674 0.188], ...
    'Name', 'Truck1');
waypoints = [-74.4551715575164 -21.7846879943097 0;
    -56.3433333333333 -21.43 0.01;
    -13.3633333333333 -21.51 0.01;
    58.9166666666667 -21.79 0.01;
    60.7666666666667 -21.4 0.01;
    61.5266666666667 -19.76 0;
    60.7666666666667 -17.78 0;
    58.7866666666667 -17.4 0.01;
    -56.8633333333333 -17.78 0.01];
speed = [30;30;30;30;30;30;30;30;30];
trajectory(truck1, waypoints, speed);

truck2 = vehicle(scenario, ...
    'ClassID', 2, ...
    'Length', 8.2, ...
    'Width', 2.5, ...
    'Height', 3.5, ...
    'Position', [-73.1366666666667 -27.05 0], ...
    'Mesh', driving.scenario.truckMesh, ...
    'PlotColor', [0.466 0.674 0.188], ...
    'Name', 'Truck2');
waypoints = [-67.5366666666667 -22.05 0;
    -56.4766666666667 -21.83 0.01;
    -13.4966666666667 -21.91 0.01;
    58.7833333333333 -22.19 0.01;
    60.6333333333333 -21.8 0.01;
    61.3933333333333 -20.16 0;
    60.6333333333333 -18.18 0;
    58.6533333333333 -17.8 0.01;
    -56.9966666666667 -18.18 0.01];
speed = [30;30;30;30;30;30;30;30;30];
trajectory(truck2, waypoints, speed);

truck3 = vehicle(scenario, ...
    'ClassID', 2, ...
    'Length', 8.2, ...
    'Width', 2.5, ...
    'Height', 3.5, ...
    'Position', [-75.7489079154881 -24.2871226311487 0], ...
    'EntryTime', 3, ...
    'Mesh', driving.scenario.truckMesh, ...
    'PlotColor', [0.466 0.674 0.188], ...
    'Name', 'Truck3');
waypoints = [-72.9289079154882 -21.9771226311487 0;
    -56.2566666666667 -21.62 0.01;
    -13.2766666666667 -21.7 0.01;
    59.0033333333333 -21.98 0.01;
    60.8533333333333 -21.59 0.01;
    61.6133333333333 -19.95 0;
    60.8533333333333 -17.97 0;
    58.8733333333333 -17.59 0.01;
    -56.7766666666667 -17.97 0.01];
speed = [30;30;30;30;30;30;30;30;30];
trajectory(truck3, waypoints, speed);

truck4 = vehicle(scenario, ...
    'ClassID', 2, ...
    'Length', 8.2, ...
    'Width', 2.5, ...
    'Height', 3.5, ...
    'Position', [-72.4811706868613 -24.5240907029193 0], ...
    'EntryTime', 1, ...
    'Mesh', driving.scenario.truckMesh, ...
    'PlotColor', [0.466 0.674 0.188], ...
    'Name', 'Truck4');
waypoints = [-69.6311706868613 -21.9440907029193 0;
    -56.4933333333333 -21.75 0.01;
    -13.5133333333333 -21.83 0.01;
    58.7666666666667 -22.11 0.01;
    60.6166666666667 -21.72 0.01;
    61.3766666666667 -20.08 0;
    60.6166666666667 -18.1 0;
    58.6366666666667 -17.72 0.01;
    -57.0133333333333 -18.1 0.01];
speed = [30;30;30;30;30;30;30;30;30];
trajectory(truck4, waypoints, speed);

truck5 = vehicle(scenario, ...
    'ClassID', 2, ...
    'Length', 8.2, ...
    'Width', 2.5, ...
    'Height', 3.5, ...
    'Position', [-77.1885048908497 -24.2846879943097 0], ...
    'EntryTime', 12, ...
    'Mesh', driving.scenario.truckMesh, ...
    'PlotColor', [0.466 0.674 0.188], ...
    'Name', 'Truck5');
waypoints = [-74.7385048908497 -21.9746879943097 0;
    -56.6266666666667 -21.62 0.01;
    -13.6466666666667 -21.7 0.01;
    58.6333333333333 -21.98 0.01;
    60.4833333333333 -21.59 0.01;
    61.2433333333333 -19.95 0;
    60.4833333333333 -17.97 0;
    58.5033333333333 -17.59 0.01;
    -57.1466666666667 -17.97 0.01];
speed = [30;30;30;30;30;30;30;30;30];
trajectory(truck5, waypoints, speed);

car1 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [44.0161986353598 42.164447149267 0.01], ...
    'Mesh', driving.scenario.carMesh, ...
    'PlotColor', [0 0.447 0.741], ...
    'Name', 'Car1');
waypoints = [44.0161986353598 42.164447149267 0.01;
    43.5333333333333 34.66 0.01;
    34.3533333333333 16.23 0.01;
    28.1933333333333 14.88 0.01;
    -8.30666666666667 9.47000000000001 0.01;
    -12.9666666666667 7.68000000000001 0.01;
    -13.8766666666667 2.71000000000001 0.01;
    -13.4966666666667 -14.89 0.01;
    -9.84666666666667 -21.28 0;
    -2.43666666666667 -22.42 0.01;
    15.2433333333333 -21.97 0.01;
    18.3433333333333 -22.79 0.01;
    19.5433333333333 -26.05 0.01;
    19.6533333333333 -52.38 0;
    20.2533333333333 -87.68 0;
    18.8733333333333 -96.1 0.01;
    15.2533333333333 -98.94 0.01;
    8.30333333333333 -100.53 0.01;
    -27.8466666666667 -101.15 0.01;
    -35.1466666666667 -101.28 0;
    -44.4466666666667 -98.68 0;
    -49.1466666666667 -86.68 0;
    -48.9466666666667 -75.18 0;
    -48.7466666666667 -62.22 0.01;
    -45.8766666666667 -53.22 0.01;
    -39.5966666666667 -50.5 0.01;
    -29.2966666666667 -51.23 0.01;
    -17.5266666666667 -52.94 0.01;
    10.9533333333333 -56.17 0.01;
    18.1733333333333 -55.61 0.01;
    20.7233333333333 -53.63 0.01;
    22.3333333333333 -47.1 0;
    22.4333333333333 -25.7 0;
    20.3333333333333 -18.7 0;
    17.4333333333333 -18 0;
    -2.74666666666666 -18.1 0.01;
    -28.6266666666667 -18.4 0.01;
    -58.0566666666667 -18.3 0.01];
speed = [30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30];
trajectory(car1, waypoints, speed);

car2 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [43.8574796957483 65.7660305230587 0], ...
    'EntryTime', 2, ...
    'Mesh', driving.scenario.carMesh, ...
    'PlotColor', [0 0.447 0.741], ...
    'Name', 'Car2');
waypoints = [43.8574796957483 65.7660305230587 0;
    43.4333333333333 34.63 0.01;
    34.2533333333333 16.2 0.01;
    28.0933333333333 14.85 0.01;
    -8.40666666666667 9.44000000000001 0.01;
    -13.0666666666667 7.65000000000001 0.01;
    -13.9766666666667 2.68000000000001 0.01;
    -13.5966666666667 -14.92 0.01;
    -9.94666666666667 -21.31 0;
    -2.53666666666668 -22.45 0.01;
    15.1433333333333 -22 0.01;
    18.2433333333333 -22.82 0.01;
    19.4433333333333 -26.08 0.01;
    19.5533333333333 -52.41 0;
    20.1533333333333 -87.71 0;
    18.7733333333333 -96.13 0.01;
    15.1533333333333 -98.97 0.01;
    8.20333333333333 -100.56 0.01;
    -27.9466666666667 -101.18 0.01;
    -35.2466666666667 -101.31 0;
    -44.5466666666667 -98.71 0;
    -49.2466666666667 -86.71 0;
    -49.0466666666667 -75.21 0;
    -48.8466666666667 -62.25 0.01;
    -45.9766666666667 -53.25 0.01;
    -39.6966666666667 -50.53 0.01;
    -29.3966666666667 -51.26 0.01;
    -17.6266666666667 -52.97 0.01;
    10.8533333333333 -56.2 0.01;
    18.0733333333333 -55.64 0.01;
    20.6233333333333 -53.66 0.01;
    22.2333333333333 -47.13 0;
    22.3333333333333 -25.73 0;
    20.2333333333333 -18.73 0;
    17.3333333333333 -18.03 0;
    -2.84666666666666 -18.13 0.01;
    -28.7266666666667 -18.43 0.01;
    -58.1566666666667 -18.33 0.01];
speed = [30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30];
trajectory(car2, waypoints, speed);

car3 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [42.2908130290817 63.9660305230587 0], ...
    'EntryTime', 4, ...
    'Mesh', driving.scenario.carMesh, ...
    'PlotColor', [0 0.447 0.741], ...
    'Name', 'Car3');
waypoints = [43.8708130290816 66.0360305230587 0;
    43.4466666666667 34.9 0.01;
    34.2666666666667 16.47 0.01;
    28.1066666666667 15.12 0.01;
    -8.39333333333334 9.71000000000001 0.01;
    -13.0533333333333 7.92 0.01;
    -13.9633333333333 2.95000000000001 0.01;
    -13.5833333333333 -14.65 0.01;
    -9.93333333333333 -21.04 0;
    -2.52333333333333 -22.18 0.01;
    15.1566666666667 -21.73 0.01;
    18.2566666666667 -22.55 0.01;
    19.4566666666667 -25.81 0.01;
    19.5666666666667 -52.14 0;
    20.1666666666667 -87.44 0;
    18.7866666666667 -95.86 0.01;
    15.1666666666667 -98.7 0.01;
    8.21666666666666 -100.29 0.01;
    -27.9333333333333 -100.91 0.01;
    -35.2333333333333 -101.04 0;
    -44.5333333333333 -98.44 0;
    -49.2333333333333 -86.44 0;
    -49.0333333333333 -74.94 0;
    -48.8333333333333 -61.98 0.01;
    -45.9633333333333 -52.98 0.01;
    -39.6833333333333 -50.26 0.01;
    -29.3833333333333 -50.99 0.01;
    -17.6133333333333 -52.7 0.01;
    10.8666666666667 -55.93 0.01;
    18.0866666666667 -55.37 0.01;
    20.6366666666667 -53.39 0.01;
    22.2466666666667 -46.86 0;
    22.3466666666667 -25.46 0;
    20.2466666666667 -18.46 0;
    17.3466666666667 -17.76 0;
    -2.83333333333332 -17.86 0.01;
    -28.7133333333333 -18.16 0.01;
    -58.1433333333333 -18.06 0.01];
speed = [30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30];
trajectory(car3, waypoints, speed);

car4 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [40.724146362415 62.1660305230587 0], ...
    'EntryTime', 6, ...
    'Mesh', driving.scenario.carMesh, ...
    'PlotColor', [0 0.447 0.741], ...
    'Name', 'Car4');
waypoints = [43.894146362415 65.8260305230587 0;
    43.47 34.69 0.01;
    34.29 16.26 0.01;
    28.13 14.91 0.01;
    -8.37000000000001 9.5 0.01;
    -13.03 7.70999999999999 0.01;
    -13.94 2.74000000000001 0.01;
    -13.56 -14.86 0.01;
    -9.91 -21.25 0;
    -2.50000000000001 -22.39 0.01;
    15.18 -21.94 0.01;
    18.28 -22.76 0.01;
    19.48 -26.02 0.01;
    19.59 -52.35 0;
    20.19 -87.65 0;
    18.81 -96.07 0.01;
    15.19 -98.91 0.01;
    8.23999999999999 -100.5 0.01;
    -27.91 -101.12 0.01;
    -35.21 -101.25 0;
    -44.51 -98.65 0;
    -49.21 -86.65 0;
    -49.01 -75.15 0;
    -48.81 -62.19 0.01;
    -45.94 -53.19 0.01;
    -39.66 -50.47 0.01;
    -29.36 -51.2 0.01;
    -17.59 -52.91 0.01;
    10.89 -56.14 0.01;
    18.11 -55.58 0.01;
    20.66 -53.6 0.01;
    22.27 -47.07 0;
    22.37 -25.67 0;
    20.27 -18.67 0;
    17.37 -17.97 0;
    -2.81 -18.07 0.01;
    -28.69 -18.37 0.01;
    -58.12 -18.27 0.01];
speed = [30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30];
trajectory(car4, waypoints, speed);

car5 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [-4.57 -114.21 0.01], ...
    'EntryTime', 8, ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car5');
waypoints = [-4.57 -114.21 0.01;
    -4.56 -96.41 0.01;
    -6.21 -65.43 0.01;
    -5.51 -57.12 0.01;
    7.9 -56.08 0;
    10.88 -56.3 0;
    18.2 -55.4 0;
    22.35 -47 0;
    22.36 -34.8 0;
    22.36 -25.28 0;
    33.01 -22.08 0.01;
    60.01 -22.08 0.01];
speed = [30;30;0;30;30;0;30;30;30;30;30;30];
waittime = [0;0;2.5;0;0;0;0;0;0;0;0;0];
trajectory(car5, waypoints, speed, waittime);

vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [35.1 27.96 0], ...
    'Yaw', -43, ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car6');

vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [33.5333333333333 26.16 0], ...
    'Yaw', -43, ...
    'Mesh', driving.scenario.carMesh, ...
    'PlotColor', [0.85 0.325 0.098], ...
    'Name', 'Car7');

vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [31.9666666666667 24.36 0], ...
    'Yaw', -43, ...
    'Mesh', driving.scenario.carMesh, ...
    'PlotColor', [0.85 0.325 0.098], ...
    'Name', 'Car8');

vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [-18.02 -44.02 0], ...
    'Yaw', -95, ...
    'Mesh', driving.scenario.carMesh, ...
    'PlotColor', [0.85 0.325 0.098], ...
    'Name', 'Car9');

vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [1.21333333333333 -69.1 0], ...
    'Yaw', 179, ...
    'Mesh', driving.scenario.carMesh, ...
    'PlotColor', [0.85 0.325 0.098], ...
    'Name', 'Car10');

vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [-23.7766666666667 -56.23 0], ...
    'Yaw', -90, ...
    'Mesh', driving.scenario.carMesh, ...
    'PlotColor', [0.85 0.325 0.098], ...
    'Name', 'Car11');

vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [-36.1933333333333 -56.3 0], ...
    'Yaw', -90, ...
    'Mesh', driving.scenario.carMesh, ...
    'PlotColor', [0.85 0.325 0.098], ...
    'Name', 'Car12');

vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [-17.34 -73.16 0], ...
    'Yaw', -1, ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car13');

vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [2.20333333333333 -81.83 0], ...
    'Yaw', 180, ...
    'Mesh', driving.scenario.carMesh, ...
    'PlotColor', [0.85 0.325 0.098], ...
    'Name', 'Car14');

vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [29.7266666666667 -49.26 0], ...
    'Yaw', -179, ...
    'Mesh', driving.scenario.carMesh, ...
    'PlotColor', [0.85 0.325 0.098], ...
    'Name', 'Car15');

vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [21.12 16.37 0], ...
    'Yaw', -171, ...
    'Mesh', driving.scenario.carMesh, ...
    'PlotColor', [255 255 0] / 255, ...
    'Name', 'Car16');

vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [13.6833333333333 15.02 0.01], ...
    'Yaw', -171, ...
    'Mesh', driving.scenario.carMesh, ...
    'PlotColor', [255 255 0] / 255, ...
    'Name', 'Car17');

vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [0.0266666666666677 13.07 0.01], ...
    'Yaw', -171, ...
    'Mesh', driving.scenario.carMesh, ...
    'PlotColor', [255 255 0] / 255, ...
    'Name', 'Car18');

vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [-16.12 -4.81 0], ...
    'Yaw', -89, ...
    'Mesh', driving.scenario.carMesh, ...
    'PlotColor', [255 255 0] / 255, ...
    'Name', 'Car19');

vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [0.333333333333334 -24.07 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'PlotColor', [255 255 0] / 255, ...
    'Name', 'Car20');

vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [6.80666666666667 -24.03 0.01], ...
    'Mesh', driving.scenario.carMesh, ...
    'PlotColor', [255 255 0] / 255, ...
    'Name', 'Car21');

vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [12.18 -23.98 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'PlotColor', [255 255 0] / 255, ...
    'Name', 'Car22');

vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [20.1333333333333 -15.32 0], ...
    'Yaw', -180, ...
    'Mesh', driving.scenario.carMesh, ...
    'PlotColor', [255 255 0] / 255, ...
    'Name', 'Car23');

vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [14.2866666666667 -15.48 0], ...
    'Yaw', -180, ...
    'Mesh', driving.scenario.carMesh, ...
    'PlotColor', [255 255 0] / 255, ...
    'Name', 'Car24');

vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [8.44 -15.36 0], ...
    'Yaw', -180, ...
    'Mesh', driving.scenario.carMesh, ...
    'PlotColor', [255 255 0] / 255, ...
    'Name', 'Car25');

vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [2.45333333333333 -15.52 0], ...
    'Yaw', -180, ...
    'Mesh', driving.scenario.carMesh, ...
    'PlotColor', [255 255 0] / 255, ...
    'Name', 'Car26');

vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [-3.45333333333333 -15.73 0], ...
    'Yaw', -180, ...
    'Mesh', driving.scenario.carMesh, ...
    'PlotColor', [255 255 0] / 255, ...
    'Name', 'Car27');

vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [16.93 -65.72 0], ...
    'Yaw', -90, ...
    'Mesh', driving.scenario.carMesh, ...
    'PlotColor', [255 255 0] / 255, ...
    'Name', 'Car28');

vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [17.3533333333333 -75.84 0], ...
    'Yaw', -90, ...
    'Mesh', driving.scenario.carMesh, ...
    'PlotColor', [255 255 0] / 255, ...
    'Name', 'Car29');

vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [17.6466666666667 -89.05 0], ...
    'Yaw', -102, ...
    'Mesh', driving.scenario.carMesh, ...
    'PlotColor', [255 255 0] / 255, ...
    'Name', 'Car30');

vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [8.31 -97.16 0], ...
    'Yaw', -178, ...
    'Mesh', driving.scenario.carMesh, ...
    'PlotColor', [255 255 0] / 255, ...
    'Name', 'Car31');

vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [2.06333333333333 -97.46 0], ...
    'Yaw', -178, ...
    'Mesh', driving.scenario.carMesh, ...
    'PlotColor', [255 255 0] / 255, ...
    'Name', 'Car32');

vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [-17.2466666666667 -97.93 0], ...
    'Yaw', -178, ...
    'Mesh', driving.scenario.carMesh, ...
    'PlotColor', [255 255 0] / 255, ...
    'Name', 'Car33');

vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [-25.4833333333333 -98.22 0], ...
    'Yaw', -178, ...
    'Mesh', driving.scenario.carMesh, ...
    'PlotColor', [255 255 0] / 255, ...
    'Name', 'Car34');

vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [-32.06 -98.3 0], ...
    'Yaw', 175, ...
    'Mesh', driving.scenario.carMesh, ...
    'PlotColor', [255 255 0] / 255, ...
    'Name', 'Car35');

vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [-39.0666666666667 -97.28 0], ...
    'Yaw', 151, ...
    'Mesh', driving.scenario.carMesh, ...
    'PlotColor', [255 255 0] / 255, ...
    'Name', 'Car36');

vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [-44.0633333333333 -92.83 0], ...
    'Yaw', 112, ...
    'Mesh', driving.scenario.carMesh, ...
    'PlotColor', [255 255 0] / 255, ...
    'Name', 'Car37');

vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [-45.77 -83.84 0], ...
    'Yaw', 92, ...
    'Mesh', driving.scenario.carMesh, ...
    'PlotColor', [255 255 0] / 255, ...
    'Name', 'Car38');

vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [-45.6866666666667 -64.67 0], ...
    'Yaw', 86, ...
    'Mesh', driving.scenario.carMesh, ...
    'PlotColor', [255 255 0] / 255, ...
    'Name', 'Car39');

vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [-19.6833333333333 -56.22 0], ...
    'Yaw', -9, ...
    'Mesh', driving.scenario.carMesh, ...
    'PlotColor', [255 255 0] / 255, ...
    'Name', 'Car40');

vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [2.73999999999999 -57.88 0], ...
    'Yaw', -5, ...
    'Mesh', driving.scenario.carMesh, ...
    'PlotColor', [255 255 0] / 255, ...
    'Name', 'Car41');
