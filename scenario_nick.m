function [allData, scenario, sensor] = scenario_nick()
%scenario_nick - Returns sensor detections
%    allData = scenario_nick returns sensor detections in a structure
%    with time for an internally defined scenario and sensor suite.
%
%    [allData, scenario, sensors] = scenario_nick optionally returns
%    the drivingScenario and detection generator objects.

% Generated by MATLAB(R) 9.9 (R2020b) and Automated Driving Toolbox 3.2 (R2020b).
% Generated on: 31-Mar-2021 00:19:56

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
    'EgoVehicleActorID', 5, ...
    'ActorProfiles', profiles);

function [scenario, egoVehicle] = createDrivingScenario
% createDrivingScenario Returns the drivingScenario defined in the Designer

% Construct a drivingScenario object.
scenario = drivingScenario;

% Add all road segments
roadCenters = [-83.7 30.5 0;
    -20.4 52.5 0;
    11.9 46.3 0;
    5.1 102.6 0;
    -80.4 79.2 0;
    -5.3 136.9 0;
    -55.6 159.7 0;
    -156.3 56.6 0;
    -127.1 -31.9 0;
    -65.3 -39.5 0;
    3.4 -17.2 0;
    31.1 6.2 0;
    -52.8 -4.5 0;
    -138.2 49.9 0;
    -106.4 75.5 0;
    -83.7 30.5 0];
laneSpecification = lanespec(4);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road');

% Add the actors
actor(scenario, ...
    'ClassID', 3, ...
    'Length', 1.7, ...
    'Width', 0.45, ...
    'Height', 1.7, ...
    'Position', [39.774 52.994 0], ...
    'Yaw', -129, ...
    'Mesh', driving.scenario.bicycleMesh, ...
    'Name', 'Bicycle3');

actor(scenario, ...
    'ClassID', 4, ...
    'Length', 0.24, ...
    'Width', 0.45, ...
    'Height', 1.7, ...
    'Position', [33.9 -41 0], ...
    'RCSPattern', [-8 -8;-8 -8], ...
    'Mesh', driving.scenario.pedestrianMesh, ...
    'PlotColor', [0.635 0.078 0.184], ...
    'Name', 'Pedestrian');

actor(scenario, ...
    'ClassID', 4, ...
    'Length', 0.24, ...
    'Width', 0.45, ...
    'Height', 1.7, ...
    'Position', [24.9 -37.2 0], ...
    'RCSPattern', [-8 -8;-8 -8], ...
    'Mesh', driving.scenario.pedestrianMesh, ...
    'PlotColor', [0 0.447 0.741], ...
    'Name', 'Pedestrian1');

actor(scenario, ...
    'ClassID', 4, ...
    'Length', 0.24, ...
    'Width', 0.45, ...
    'Height', 1.7, ...
    'Position', [30.9 -33.6 0], ...
    'RCSPattern', [-8 -8;-8 -8], ...
    'Mesh', driving.scenario.pedestrianMesh, ...
    'PlotColor', [0.85 0.325 0.098], ...
    'Name', 'Pedestrian2');

% Add the ego vehicle
egoVehicle = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [-14.92 -29.96 0.01], ...
    'Mesh', driving.scenario.carMesh, ...
    'PlotColor', [0.494 0.184 0.556], ...
    'Name', 'Car');
waypoints = [-14.92 -29.96 0.01;
    2.03 -22.87 0.01;
    20.7 -14.2 0;
    27.8 -9.5 0;
    33.2 -0.7 0;
    35.1 6.7 0;
    35.7 12.8 0;
    34.5 17.8 0;
    31.8 23.6 0;
    27.7 28.3 0;
    21.97 32.05 0.01;
    11.55 35.18 0.01;
    3.79 35.39 0.01;
    -2.93 34.35 0.01;
    -10.84 31.9 0.01;
    -18.23 28.41 0.01;
    -23.7 24.66 0.01;
    -31.67 18.1 0.01;
    -38.66 12.73 0.01;
    -48.24 4.55 0.01;
    -58.19 -0.97 0.01;
    -67.25 -4.25 0.01;
    -77.46 -6.17 0.01;
    -85.11 -6.23 0.01;
    -93.45 -5.08 0.01;
    -107.31 0.84 0.01;
    -116.98 8.09 0.01;
    -124.93 17.52 0.01;
    -130.07 27.87 0.01;
    -132.75 41.04 0.01;
    -131.82 54.84 0.01;
    -125.63 67.02 0.01;
    -116.19 72.16 0.01;
    -109.44 70.75 0.01;
    -104.93 66.66 0.01;
    -102.47 61.31 0.01;
    -100.78 55.4 0.01;
    -98.59 45.4 0.01;
    -95.56 36.67 0.01;
    -89.65 28.01 0.01;
    -83.17 24.06 0.01;
    -76.41 22.02 0.01;
    -70.71 21.81 0.01;
    -59.18 24.55 0.01;
    -48.49 30.9 0.01;
    -38.23 38.72 0.01;
    -32.15 42.91 0.01;
    -14.34 46.68 0.01;
    -4.77 44.1 0.01;
    2.14 42.21 0.01;
    12.96 40.67 0.01;
    34.4 49.75 0.01;
    43.69 66.85 0.01;
    44.03 78.17 0.01;
    37.26 95.28 0.01;
    22.45 105.89 0.01;
    4.09 107.71 0.01;
    -10.8 102.3 0;
    -27.7 89.5 0;
    -41.1 78.4 0;
    -54.1 71.8 0;
    -69.9 74.2 0;
    -74 89.9 0;
    -63.7 100 0;
    -54.3 103.7 0;
    -43.8 105.7 0;
    -33.2 107.7 0;
    -23.9 110.1 0;
    -10.6 117.5 0;
    -1.6 129.6 0;
    0 139.6 0;
    -2.1 149.5 0;
    -9.8 159.5 0;
    -24.1 167.1 0;
    -37.6 168.2 0;
    -51 166.2 0;
    -62.3 163.1 0;
    -71 160 0;
    -89.5 151.7 0;
    -106.1 141 0;
    -120.4 129.5 0;
    -131 118.3 0;
    -143.3 102.2 0;
    -148.6 92.3 0;
    -154.5 80.5 0;
    -159.4 66.2 0;
    -162.6 53 0;
    -164 41.5 0;
    -164.5 30.8 0;
    -163.4 18.7 0;
    -160.9 6.3 0;
    -156 -7.3 0;
    -146.7 -22.3 0;
    -135.7 -33.1 0;
    -121.7 -40.9 0;
    -110 -44.8 0;
    -99 -46.6 0;
    -86.86 -47.59 0.01;
    -71.1 -46.1 0;
    -58.1 -43.8 0;
    -48 -41.3 0;
    -30 -35.7 0;
    -21.5 -32.4 0;
    -13.16 -29.36 0.01];
speed = [30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30;30];
trajectory(egoVehicle, waypoints, speed);

vehicle(scenario, ...
    'ClassID', 6, ...
    'Length', 5, ...
    'Width', 5, ...
    'Height', 10, ...
    'Position', [26.43 68.15 0], ...
    'Yaw', -24, ...
    'PlotColor', [255 0 255] / 255, ...
    'Name', 'small building');

vehicle(scenario, ...
    'ClassID', 6, ...
    'Length', 5, ...
    'Width', 5, ...
    'Height', 10, ...
    'Position', [12.5433333333333 58.03 0], ...
    'Yaw', -87, ...
    'PlotColor', [255 0 255] / 255, ...
    'Name', 'small building1');

vehicle(scenario, ...
    'ClassID', 6, ...
    'Length', 5, ...
    'Width', 5, ...
    'Height', 10, ...
    'Position', [24.5966666666667 83.06 0], ...
    'Yaw', 34, ...
    'PlotColor', [255 0 255] / 255, ...
    'Name', 'small building2');

vehicle(scenario, ...
    'ClassID', 6, ...
    'Length', 5, ...
    'Width', 5, ...
    'Height', 10, ...
    'Position', [14 90.28 0], ...
    'Yaw', 76, ...
    'PlotColor', [255 0 255] / 255, ...
    'Name', 'small building3');

actor(scenario, ...
    'ClassID', 5, ...
    'Length', 2.4, ...
    'Width', 0.76, ...
    'Height', 0.8, ...
    'Position', [-14.22 95.94 0.01], ...
    'Yaw', -50, ...
    'PlotColor', [166 166 166] / 255, ...
    'Name', 'Barrier');

vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [-12.65 97.07 0.01], ...
    'Yaw', 29, ...
    'Mesh', driving.scenario.carMesh, ...
    'PlotColor', [0.466 0.674 0.188], ...
    'Name', 'Car1');

car2 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [-16.4866666666667 -31.76 0.01], ...
    'Mesh', driving.scenario.carMesh, ...
    'PlotColor', [0.494 0.184 0.556], ...
    'Name', 'Car2');
waypoints = [-16.4866666666667 -26.76 0.01;
    0.463333333333333 -19.67 0.01;
    19.1333333333333 -11 0;
    26.2333333333333 -6.3 0;
    31.6333333333333 2.5 0;
    33.5333333333333 9.9 0;
    32.77 14.53 0.01;
    30.31 19.93 0.01;
    25.32 25.54 0.01;
    20.4 29.02 0.01;
    16.88 30.29 0.01;
    9.27 31.77 0.01;
    1.17 31.2 0.01;
    -3.62 30.22 0.01;
    -11.94 27.26 0.01;
    -16.23 25 0.01;
    -22.57 20.56 0.01;
    -31.87 13.31 0.01;
    -36.8 8.73 0.01;
    -46.38 1.19 0.01;
    -57.44 -4.8 0.01;
    -71.95 -8.67 0.01;
    -81.32 -9.87 0.01;
    -91.7 -8.56 0.01;
    -99.37 -6.21 0.01;
    -111.8 0.39 0.01;
    -121.89 8.99 0.01;
    -130.12 20.21 0.01;
    -135.09 32.71 0.01;
    -136.67 45.07 0.01;
    -134.75 58.99 0.01;
    -127.196666666667 70.22 0.01;
    -117.756666666667 75.36 0.01;
    -109.83 74.9 0.01;
    -103.86 70.78 0.01;
    -100.38 65.45 0.01;
    -98.18 59.28 0.01;
    -95.76 48.69 0.01;
    -92.93 39.03 0.01;
    -88.52 32.07 0.01;
    -84.19 27.95 0.01;
    -77.73 25.75 0.01;
    -71.98 25.39 0.01;
    -60.7466666666667 27.75 0.01;
    -50.0566666666667 34.1 0.01;
    -39.7966666666667 41.92 0.01;
    -33.7166666666667 46.11 0.01;
    -15.9066666666667 49.88 0.01;
    -6.23 48.19 0.01;
    0.75 46.28 0.01;
    11.6 44.31 0.01;
    32.45 53.12 0.01;
    40.41 67.98 0.01;
    40.2 80.1 0.01;
    32.17 95.67 0.01;
    18.15 103.85 0.01;
    2.86 103.84 0.01;
    -7.49 95.81 0.01;
    -19.4 91.43 0.01;
    -29.13 83.13 0.01;
    -39.91 74.96 0.01;
    -56.25 67.84 0.01;
    -74.43 73.33 0.01;
    -76.75 93.06 0.01;
    -65.2666666666667 103.2 0;
    -55.8666666666667 106.9 0;
    -45.3666666666667 108.9 0;
    -34.7666666666667 110.9 0;
    -25.4666666666667 113.3 0;
    -12.1666666666667 120.7 0;
    -4.55 131.67 0.01;
    -3.29 140.76 0.01;
    -5.68 149.49 0.01;
    -13.43 158.09 0.01;
    -27.59 163.93 0.01;
    -38.79 164.85 0.01;
    -50.91 162.95 0.01;
    -63.18 159.63 0.01;
    -71.7 156.25 0.01;
    -89 148.07 0.01;
    -105.05 137.92 0.01;
    -118.9 126.91 0.01;
    -129.69 114.84 0.01;
    -139.43 101.63 0.01;
    -146.24 90.12 0.01;
    -151.43 79.04 0.01;
    -156.11 64.91 0.01;
    -159.09 54.54 0.01;
    -160.87 42.47 0.01;
    -161.22 28.98 0.01;
    -160.66 19.67 0.01;
    -158.09 7.75 0.01;
    -153.98 -3.55 0.01;
    -145.8 -17.4 0.01;
    -135.44 -28.27 0.01;
    -122.79 -36.44 0.01;
    -111.5 -40.41 0.01;
    -100.56 -42.69 0.01;
    -88.1666666666667 -44 0;
    -72.6666666666667 -42.9 0;
    -59.6666666666667 -40.6 0;
    -49.5666666666667 -38.1 0;
    -31.5666666666667 -32.5 0;
    -23.0666666666667 -29.2 0;
    -14.7266666666667 -26.16 0.01];
speed = [20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20];
trajectory(car2, waypoints, speed);

actor(scenario, ...
    'ClassID', 7, ...
    'Length', 50, ...
    'Width', 20, ...
    'Height', 30, ...
    'Position', [-43.38 137.22 0], ...
    'Yaw', 5, ...
    'PlotColor', [0.635 0.078 0.184], ...
    'Name', 'BIG BUILDING');

actor(scenario, ...
    'ClassID', 7, ...
    'Length', 50, ...
    'Width', 20, ...
    'Height', 30, ...
    'Position', [-95.1466666666667 114.22 0], ...
    'Yaw', 42, ...
    'Name', 'BIG BUILDING1');

actor(scenario, ...
    'ClassID', 7, ...
    'Length', 50, ...
    'Width', 20, ...
    'Height', 30, ...
    'Position', [-109.413333333333 35.12 0], ...
    'Yaw', -68, ...
    'PlotColor', [0.635 0.078 0.184], ...
    'Name', 'BIG BUILDING2');

vehicle(scenario, ...
    'ClassID', 6, ...
    'Length', 5, ...
    'Width', 5, ...
    'Height', 10, ...
    'Position', [-80.4233333333333 -21.5 0], ...
    'Yaw', -91, ...
    'PlotColor', [255 0 255] / 255, ...
    'Name', 'small building4');

vehicle(scenario, ...
    'ClassID', 6, ...
    'Length', 5, ...
    'Width', 5, ...
    'Height', 10, ...
    'Position', [-51.88 24.39 0], ...
    'Yaw', -57, ...
    'PlotColor', [255 0 255] / 255, ...
    'Name', 'small building5');

vehicle(scenario, ...
    'ClassID', 6, ...
    'Length', 5, ...
    'Width', 5, ...
    'Height', 10, ...
    'Position', [-66.7266666666667 -19.85 0], ...
    'Yaw', -76, ...
    'PlotColor', [255 0 255] / 255, ...
    'Name', 'small building6');

vehicle(scenario, ...
    'ClassID', 6, ...
    'Length', 5, ...
    'Width', 5, ...
    'Height', 10, ...
    'Position', [-41.8533333333333 31.23 0], ...
    'Yaw', -53, ...
    'PlotColor', [255 0 255] / 255, ...
    'Name', 'small building7');

vehicle(scenario, ...
    'ClassID', 6, ...
    'Length', 5, ...
    'Width', 5, ...
    'Height', 10, ...
    'Position', [-53.16 -15.41 0], ...
    'Yaw', -67, ...
    'PlotColor', [255 0 255] / 255, ...
    'Name', 'small building8');

vehicle(scenario, ...
    'ClassID', 6, ...
    'Length', 5, ...
    'Width', 5, ...
    'Height', 10, ...
    'Position', [-32.6766666666667 38.55 0], ...
    'Yaw', -54, ...
    'PlotColor', [255 0 255] / 255, ...
    'Name', 'small building9');

vehicle(scenario, ...
    'ClassID', 6, ...
    'Length', 5, ...
    'Width', 5, ...
    'Height', 10, ...
    'Position', [-43.1433333333333 -9.56999999999999 0], ...
    'Yaw', -57, ...
    'PlotColor', [255 0 255] / 255, ...
    'Name', 'small building10');

vehicle(scenario, ...
    'ClassID', 6, ...
    'Length', 5, ...
    'Width', 5, ...
    'Height', 10, ...
    'Position', [-20.25 43.65 0], ...
    'Yaw', -85, ...
    'PlotColor', [255 0 255] / 255, ...
    'Name', 'small building11');

vehicle(scenario, ...
    'ClassID', 6, ...
    'Length', 5, ...
    'Width', 5, ...
    'Height', 10, ...
    'Position', [-32.9666666666667 -1.52 0], ...
    'Yaw', -51, ...
    'PlotColor', [255 0 255] / 255, ...
    'Name', 'small building12');

vehicle(scenario, ...
    'ClassID', 6, ...
    'Length', 5, ...
    'Width', 5, ...
    'Height', 10, ...
    'Position', [-63.0433333333333 19.59 0], ...
    'Yaw', -76, ...
    'PlotColor', [255 0 255] / 255, ...
    'Name', 'small building13');

vehicle(scenario, ...
    'ClassID', 6, ...
    'Length', 5, ...
    'Width', 5, ...
    'Height', 10, ...
    'Position', [-94.4 -20.33 0], ...
    'Yaw', -104, ...
    'PlotColor', [255 0 255] / 255, ...
    'Name', 'small building14');

vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [-114.89 138.07 0], ...
    'Yaw', 39, ...
    'Mesh', driving.scenario.carMesh, ...
    'PlotColor', [0.929 0.694 0.125], ...
    'Name', 'Car3');

vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [-127.806666666667 126.19 0], ...
    'Yaw', 44, ...
    'Mesh', driving.scenario.carMesh, ...
    'PlotColor', [0.929 0.694 0.125], ...
    'Name', 'Car4');

vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [-95.8833333333333 151.23 0], ...
    'Yaw', 30, ...
    'Mesh', driving.scenario.carMesh, ...
    'PlotColor', [0.929 0.694 0.125], ...
    'Name', 'Car5');

vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [-67.63 164.61 0], ...
    'Yaw', 21, ...
    'Mesh', driving.scenario.carMesh, ...
    'PlotColor', [0.929 0.694 0.125], ...
    'Name', 'Car6');

vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [-10.8966666666667 113.07 0], ...
    'Yaw', -150, ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car7');

vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [-36.6333333333333 103.77 0], ...
    'Yaw', -169, ...
    'Mesh', driving.scenario.carMesh, ...
    'PlotColor', [0.929 0.694 0.125], ...
    'Name', 'Car8');
