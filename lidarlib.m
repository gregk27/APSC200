function [cuboids, cloud] = lidarlib(ptCloud, vehicle, varargin)
    persistent fig;

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
    addParameter(p, 'callback', @defaultCallback);
    addParameter(p, 'inertial', false);
    addParameter(p, 'plot', '', plotModes);
    % Get input arguments
    parse(p, varargin{:});
    
    % Initialise figure if missing
    if isempty(fig) && ismember(plotModes, p.Results.plot)
        fig = findall(0, 'Type', 'Figure', 'Tag', 'lidar');
        if isempty(fig)
            fig = figure('name', 'LIDAR Result', 'Tag', 'lidar');
        end
        figure(fig);
        % Mark vehicle location
        plot(cuboidModel([1,0,1,1,1,0.5,0,0,0]));
    end

    % Filter out road surface
    [model1,inlierIndices,outlierIndices] = pcfitplane(ptCloud,1,[0,0,1],0.02);
    cloud = select(ptCloud,inlierIndices);
    
    % Eliminate points on vehicle roof
    magnitudes = sqrt((cloud.Location(:, 1)-1).^2+cloud.Location(:, 2).^2);
    cloud = pointCloud(cloud.Location(magnitudes > 1, :));
    
    % Cluster to get cuboids
    [labels, numClusters] = pcsegdist(cloud, p.Results.minDist);
    
    % Draw point clouds if enabled
    if ismember(plotModes, p.Results.plot)
        pcshow(cloud.Location, labels);
        title(sprintf('Point Cloud Clusters @ %i',scenario.SimulationTime));
    end
    
    cuboids = [];
    
    % Get and plot cuboids5 from point clouds
    for i = 1:numClusters
        idx = find(labels == i);
        model = pcfitcuboid(cloud, idx);
        if p.Results.plot == 'all'
            plot(model)
        end
        if prod(model.Dimensions) > p.Results.minSize && prod(model.Dimensions) < p.Results.maxSize
            % Narrow down to vehicles in specific area
            if model.Center(1) > p.Results.minX && model.Center(1) < p.Results.maxX && model.Center(2) > -p.Results.maxZ && model.Center(2) < -p.Results.minZ
                if p.Results.plot == 'filtered'
                    plot(model)
                end
                inertial = cuboid2Inertial(model, vehicle);
                res = p.Results.callback(model, inertial);
                if res
                    if p.Results.plot == 'selected'
                        plot(model)
                    end
                    cuboids = [cuboids, inertial];
                end
            end
        end
    end
end

function [res] = defaultCallback(model, inertial)
    res = true;
end
