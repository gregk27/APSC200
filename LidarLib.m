classdef LidarLib
% LIDARLIB  Library for lidar processing
% Functions include cuboid detection and conversion from vehicle to
% inertial space
%
%
% PROCESS  Find cuboids in lidar point cloud
%
% Required Parameters 
%  - ptCloud: Lidar point cloud output
%  - scenario: The scenario variable
%  - vehicle: The ego vehicle
%
% PROCESS  Find cuboids in lidar point cloud
%
% Required Parameters 
%  - ptCloud: Lidar point cloud output
%  - scenario: The scenario variable
%  - vehicle: The ego vehicle
%
% Optional parameters
%  - minDist: Minimum distance from the ego vehicle, used to remove
%      vehicle from results [default: 0.05]
%  - minSize: Minimum size of cuboid [default: 0]
%  - maxSize: Maximum size of cuboid [default: 999]
%  - minX: Minumum x position (forward positive) [default: -999]
%  - maxX: Maxuimum x position (forward positive) [default: 999]
%  - minY: Minimum y position (left positive) [default: -999]
%  - maxY: Maximum y position (left positive) [default: 999]
%  - inertial: Flag to indicate if cuboids should be in inertial space
%      instead of vehicle space [default: false]
%  - plot: Plotting mode [default: ''], options are:
%      -> '': Do not plot
%      -> 'cloud': Only plot point cloud
%      -> 'all': Point all cuboids
%      -> 'filtered': Plot only cuboids meeting filter results
%      -> 'selected': Plot only cuboids meeting callback (same as
%            filtered if no callback)
%  - roi: List representing the Region of Interest, in the format [xmin, xman, ymin, ymax, zmin, zmax]. 
%      This filter will be applied before any others for performance. All coordinates same as plot [default: []]
%  - callback: Callback function to be called when other parameters met, 
%      needs signagure: function [res] = callback(model, inertial). [default: returns true]
%      -> res is result, if true cuboid will be returned.
%      -> model is cuboid in vehicle space
%      -> inertial is cuboid in interial space (using cuboid2Inertial)
%
% Returns
%  - cuboids: List of cuboidModels which fit filters and callback 
%  - cloud: Point cloud with ground and min distance removed
%  - fig: Figure used when plotting, empty if plot=''
%
% Example
%   cuboids = PROCESS(ptCloud, scenario, egoVehicle, 'maxSize', 100, 'minX', 0, 'minY', 0, 'maxY', 5, 'plot', 'filtered')
%     will return all cuboids under 100m², infront of vehicle and less
%     than 5m to the left. Additionally, it will plot these cuboids
%
% See also: LIDARLIB.CUBOID2INERTIAL
%
%
% CUBOID2INERTIAL  Converts cuboid from vehicle space to inertial space
% 
% Parameters
%  - cuboid: The cuboid to convert
%  - egoVehicle: The vehicle which observed the cuboid
% 
% Returns
%  - cuboid: The cuboid in inertial space
%
% See also: VEHICLE2INERTIAL
% Based on: https://www.mathworks.com/help/vision/ug/track-vehicles-using-lidar.html
%
    methods(Static)
        function [cuboids, cloud, fig] = process(ptCloud, scenario, vehicle, varargin)
            % PROCESS  Find cuboids in lidar point cloud
            %
            % Required Parameters 
            %  - ptCloud: Lidar point cloud output
            %  - scenario: The scenario variable
            %  - vehicle: The ego vehicle
            %
            % Optional parameters
            %  - minDist: Minimum distance from the ego vehicle, used to remove
            %      vehicle from results [default: 0.05]
            %  - minSize: Minimum size of cuboid [default: 0]
            %  - maxSize: Maximum size of cuboid [default: 999]
            %  - minX: Minumum x position (forward positive) [default: -999]
            %  - maxX: Maxuimum x position (forward positive) [default: 999]
            %  - minY: Minimum y position (left positive) [default: -999]
            %  - maxY: Maximum y position (left positive) [default: 999]
            %  - inertial: Flag to indicate if cuboids should be in inertial space
            %      instead of vehicle space [default: false]
            %  - plot: Plotting mode [default: ''], options are:
            %      -> '': Do not plot
            %      -> 'cloud': Only plot point cloud
            %      -> 'all': Point all cuboids
            %      -> 'filtered': Plot only cuboids meeting filter results
            %      -> 'selected': Plot only cuboids meeting callback (same as
            %            filtered if no callback)
            %  - roi: List representing the Region of Interest, in the format [xmin, xman, ymin, ymax, zmin, zmax]. 
            %      This filter will be applied before any others for performance. All coordinates same as plot [default: []]
            %  - callback: Callback function to be called when other parameters met, 
            %      needs signagure: function [res] = callback(model, inertial). [default: returns true]
            %      -> res is result, if true cuboid will be returned.
            %      -> model is cuboid in vehicle space
            %      -> inertial is cuboid in interial space (using cuboid2Inertial)
            %
            % Returns
            %  - cuboids: List of cuboidModels which fit filters and callback 
            %  - cloud: Point cloud with ground and min distance removed
            %  - fig: Figure used when plotting, empty if plot=''
            %
            % Example
            %   cuboids = PROCESS(ptCloud, scenario, egoVehicle, 'maxSize', 100, 'minX', 0, 'minY', 0, 'maxY', 5, 'plot', 'filtered')
            %     will return all cuboids under 100m², infront of vehicle and less
            %     than 5m to the left. Additionally, it will plot these cuboids
            %
            % See also: LIDARLIB.CUBOID2INERTIAL


            plotModes = {'cloud', 'all', 'filtered', 'selected'};

            p = inputParser;
            % Initialise input arguments
            addParameter(p, 'minDist', 0.5);
            addParameter(p, 'minSize', 0);
            addParameter(p, 'maxSize', 999);
            addParameter(p, 'minX', -999);
            addParameter(p, 'maxX', 999);
            addParameter(p, 'minY', -999);
            addParameter(p, 'maxY', 999);
            addParameter(p, 'inertial', true);
            addParameter(p, 'plot', '');
            addParameter(p, 'roi', []);
            addParameter(p, 'callback', @defaultCallback);
            % Get input arguments
            parse(p, varargin{:});
            
            % Apply ROI filter if desired
            if numel(p.Results.roi) == 6
                ptCloud = select(ptCloud, findPointsInROI(ptCloud, p.Results.roi));
            end
            

            % Filter out road surface
            [model1,inlierIndices,outlierIndices] = pcfitplane(ptCloud,1,[0,0,1],0.02);
            cloud = select(ptCloud,inlierIndices);

            % Eliminate points on vehicle roof
            magnitudes = sqrt((cloud.Location(:, 1)-1).^2+cloud.Location(:, 2).^2);
            cloud = pointCloud(cloud.Location(magnitudes > 1, :));

            % Cluster to get cuboids
            [labels, numClusters] = pcsegdist(cloud, p.Results.minDist);

            
            fig = [];
            % Draw point clouds if enabled
            if ismember(p.Results.plot, plotModes)
                % Initialise figure if missing
                fig = findall(0, 'Type', 'Figure', 'Tag', 'lidar');
                if isempty(fig)
                    fig = figure('name', 'LIDAR Result', 'Tag', 'lidar');
                end
                figure(fig);
            end
            
            if isempty(cloud.Location)
                cuboids = [];
                return;
            end
            
            if ~isempty(fig)
                % Draw point cloud
                pcshow(cloud.Location, labels);
                title(sprintf('Point Cloud Clusters @ %i',scenario.SimulationTime));
                xlabel('X');
                ylabel('Y');
                zlabel('Z');

                % Mark vehicle location
                plot(cuboidModel([1,0,1,1,1,0.5,0,0,0]));
            end

            cuboids = [];

            % Get and plot cuboids5 from point clouds
            for i = 1:numClusters
                idx = find(labels == i);
                model = pcfitcuboid(cloud, idx);
                if strcmp(p.Results.plot, 'all')
                    figure(fig);
                    plot(model);
                end
                if prod(model.Dimensions) > p.Results.minSize && prod(model.Dimensions) < p.Results.maxSize
                    % Narrow down to vehicles in specific area
                    if model.Center(1) > p.Results.minX && model.Center(1) < p.Results.maxX && model.Center(2) < p.Results.maxY && model.Center(2) > p.Results.minY
                        if strcmp(p.Results.plot, 'filtered')
                            figure(fig);
                            plot(model);
                        end
                        inertial = LidarLib.cuboid2Inertial(model, vehicle);
                        res = p.Results.callback(model, inertial);
                        if res
                            if strcmp(p.Results.plot, 'selected')
                                figure(fig);
                                plot(model);
                            end
                            cuboids = [cuboids, inertial];
                        end
                    end
                end
            end
        end

        function [cuboid] = cuboid2Inertial(cuboid, egoVehicle)
            % CUBOID2INERTIAL  Converts cuboid from vehicle space to inertial space
            % 
            % Parameters
            %  - cuboid: The cuboid to convert
            %  - egoVehicle: The vehicle which observed the cuboid
            % 
            % Returns
            %  - cuboid: The cuboid in inertial space
            %
            % See also: VEHICLE2INERTIAL

            params = cuboid.Parameters;
            positions = params(1:3).';

            % Euler angles defining orientation of local axes
            yaw = -egoVehicle.Yaw;
            pitch = egoVehicle.Pitch;
            roll =  egoVehicle.Roll;

            % Create orientation matrix from Euler angles using quaternion class
            q = quaternion([yaw pitch roll],'eulerd','zyx','frame');
            myRotationMatrix = rotmat(q,'frame');

            posInertial = myRotationMatrix*positions + egoVehicle.Position';

            % Shift angles based on vehicle orientation
            angles = params(7:9) + [pitch roll -yaw];    

            params = [ posInertial.' params(4:6) angles ];
            cuboid = cuboidModel(params);
        end
    end
    methods(Static, Hidden)
        function [res] = defaultCallback(model, inertial)
            res = true;
        end
    end
end
