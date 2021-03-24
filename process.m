function [] = process(scenario, objects, ptCloud, vehicle)
    persistent fig;
    persistent detected;
    persistent textField;
    persistent hTopViewAxes;
    if isempty(fig)
        fig = findall(0, 'Type', 'Figure', 'Tag', 'lidar');
        if isempty(fig)
            fig = figure('name', 'LIDAR Result', 'Tag', 'lidar');
        end
        detected = [];
        [textField, hTopViewAxes] = plotScenario(scenario, vehicle);
    end

    % objectsStruct = [objects{:}];
    
     %if ~isempty(objects)
     %        allPosInertial = vehicle2Inertial(objects, vehicle);
     %        disp('Class ids: %i\n');
     %        disp([objectsStruct.ObjectClassID]);

     %end

    % plot3(allPosInertial(1,:), allPosInertial(2,:), allPosInertial(3,:), 'b. ', 'Parent', hTopViewAxes);
    
    % Filter out road surface
    [model1,inlierIndices,outlierIndices] = pcfitplane(ptCloud,1,[0,0,1],0.02);
    cloud = select(ptCloud,inlierIndices);
    
    % Eliminate points on vehicle roof
    magnitudes = sqrt((cloud.Location(:, 1)-1).^2+cloud.Location(:, 2).^2);
    cloud = pointCloud(cloud.Location(magnitudes > 1, :));
    
    % Cluster to get cuboids
    [labels, numClusters] = pcsegdist(cloud, 0.5);
        
    % Draw point clouds
    figure(fig);
    pcshow(cloud.Location, labels);
    title(sprintf('Point Cloud Clusters @ %i',scenario.SimulationTime));
    
    % Mark vehicle location
    plot(cuboidModel([1,0,1,1,1,0.5,0,0,0]));
     
    closest = [];

    % Get and plot cuboids5 from point clouds
    for i = 1:numClusters
        idx = find(labels == i);
        model = pcfitcuboid(cloud, idx);
        if prod(model.Dimensions) > 5
            %plot(model)
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
        figure(fig);
        plot(closest);
        ws = cuboid2Inertial(closest, vehicle);
        if isempty(detected)
            detected = [ws.Center];
            axes(hTopViewAxes);
            plot(ws);
        else
            rows = ismembertol(detected, ws.Center, 0.15, 'ByRows', true);
            if ~rows
                detected = [detected; ws.Center];
                axes(hTopViewAxes);
                plot(ws);
            else
                disp(detected);
                disp(rows);
                disp(detected(rows, :));
                detected(rows, :) = ws.Center;
            end
        end
    end
        
    % Update message
    s = size(detected);
    message = sprintf('Number of vehicles detected: %d\n', s(1));
    textField.String = message;
end

