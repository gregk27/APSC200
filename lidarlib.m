function [cuboids, cloud] = lidarlib(ptCloud, vehicle, varargin)

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
    % Get input arguments
    parse(p, varargin{:});

    % Filter out road surface
    [model1,inlierIndices,outlierIndices] = pcfitplane(ptCloud,1,[0,0,1],0.02);
    cloud = select(ptCloud,inlierIndices);
    
    % Eliminate points on vehicle roof
    magnitudes = sqrt((cloud.Location(:, 1)-1).^2+cloud.Location(:, 2).^2);
    cloud = pointCloud(cloud.Location(magnitudes > 1, :));
    
    % Cluster to get cuboids
    [labels, numClusters] = pcsegdist(cloud, p.Results.minDist);
    
    cuboids = [];
    
    % Get and plot cuboids5 from point clouds
    for i = 1:numClusters
        idx = find(labels == i);
        model = pcfitcuboid(cloud, idx);
        if prod(model.Dimensions) > p.Results.minSize && prod(model.Dimensions) < p.Results.maxSize
            % Narrow down to vehicles in specific area
            if model.Center(1) > p.Results.minX && model.Center(1) < p.Results.maxX && model.Center(2) > -p.Results.maxZ && model.Center(2) < -p.Results.minZ
                inertial = cuboid2Inertial(model, vehicle);
                res = p.Results.callback(model, inertial);
                if res
                    cuboids = [cuboids, inertial];
                end
            end
        end
    end
end

function [res] = defaultCallback(model, inertial)
    res = true;
end
