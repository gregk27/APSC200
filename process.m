function [] = process(scenario, objects, ptCloud, world, vehicle)
    persistent fig
    if isempty(fig)
        fig = figure;
    end

    % objectsStruct = [objects{:}];
    
     %if ~isempty(objects)
     %        allPosInertial = vehicle2Inertial(objects, vehicle);
     %        disp('Class ids: %i\n');
     %        disp([objectsStruct.ObjectClassID]);

     %end

    % plot3(allPosInertial(1,:), allPosInertial(2,:), allPosInertial(3,:), 'b. ', 'Parent', hTopViewAxes);
    
    % Cluster point clouds
    [model1,inlierIndices,outlierIndices] = pcfitplane(ptCloud,1,[0,0,1],0.02);
    cloud = select(ptCloud,inlierIndices);
    [labels, numClusters] = pcsegdist(cloud, 0.5);
    
    % Draw point clouds
    figure(fig);
    pcshow(cloud.Location, labels);
    title(sprintf('Point Cloud Clusters @ %i',scenario.SimulationTime));
     
    closest = [];

    % Get and plot cuboids5 from point clouds
    for i = 1:numClusters
        idx = find(labels == i);
        model = pcfitcuboid(cloud, idx);
        if prod(model.Dimensions) > 5
            %plot(model)
            axes(world);
            plot(cuboid2Inertial(model, vehicle));
            % Narrow down to vehicles in specific area
            if model.Center(1) > 0 && model.Center(2) < -0.5 && model.Center(2) > -2
                % If it's closer than closest, use it
                if isempty(closest)
                    closest = model;
                elseif model.Center(1) < closest.Center(1)
                    closest = model;
                end
            end
            %plot(model.Center)
        end
    end
    %plot3(closest.Center(1), closest.Center(2), closest.Center(3));
    if ~isempty(closest)
        figure(fig)
        plot(closest)
    end
        
    % Update message
    message = sprintf('Number of objects sampled in one time step: %f\n', length(objects));
    textField.String = message;
end

