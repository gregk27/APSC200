function [] = process(scenario, objects, ptCloud)
    persistent fig
    if isempty(fig)
        fig = figure;
    end

    % objectsStruct = [objects{:}];
    
    % if ~isempty(objects)
    %         allPosInertial = vehicle2Inertial(objects, egoVehicle);
    %         disp('Class ids: %i\n');
    %         disp([objectsStruct.ObjectClassID]);

    % end

    % plot3(allPosInertial(1,:), allPosInertial(2,:), allPosInertial(3,:), 'b. ', 'Parent', hTopViewAxes);
    
    % Cluster point clouds
    [model1,inlierIndices,outlierIndices] = pcfitplane(ptCloud,1,[0,0,1],0.02);
    cloud = select(ptCloud,inlierIndices);
    [labels, numClusters] = pcsegdist(cloud, 0.5);
    
    % Draw point clouds
    figure(fig);
    pcshow(cloud.Location, labels);
    title(sprintf('Point Cloud Clusters @ %i',scenario.SimulationTime));
    
    % Get and plot cuboids from point clouds
    for i = 1:numClusters
        idx = find(labels == i);
        model = pcfitcuboid(cloud, idx);
        plot(model)
    end
    
    % Update message
    message = sprintf('Number of objects sampled in one time step: %f\n', length(objects));
    textField.String = message;

end

