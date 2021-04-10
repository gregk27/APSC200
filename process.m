function [] = process(scenario, objects, ptCloud, vehicle)
    persistent detected;
    persistent textField;
    persistent hTopViewAxes;
    persistent hChaseViewAxes;
    persistent conn;
    global closest;
    global plates;
    if isempty(textField)
        detected = [];
        plates = [];
        [textField, hTopViewAxes, hChaseViewAxes] = plotScenario(scenario, vehicle);
    end
    if isempty(conn)
        conn = dbconn();
    end

    % plot3(allPosInertial(1,:), allPosInertial(2,:), allPosInertial(3,:), 'b. ', 'Parent', hTopViewAxes);
    
    closest = [];

    [cuboids, cloud, fig] = LidarLib.process(ptCloud, scenario, vehicle, 'minSize', 5, 'maxSize', 35, 'minX', 0, 'maxX', 12.5, 'maxY', -0.5, 'minY', -3,...
        'minYaw', -60, 'maxYaw', 60, 'minRatio', 1, 'maxRatio', 3, 'plot', 'filtered', 'callback', @onFilter, 'roi', [-1, 30, -10, 0.5, 0, 5]);

    if ~isempty(closest)
        figure(fig);
        plot(closest);
        % Get vehicle position in interial (world) space
        ws = LidarLib.cuboid2Inertial(closest, vehicle);
        % If detections are empty then add it
        if isempty(detected)
            detected = [ws.Center];
            axes(hTopViewAxes);
            plot(ws);
            
            axes(hChaseViewAxes);
            plot(cuboidModel([ws.Center+[0,0,2], 0.2, 0.2, 1, 0, 0, 0]));
        else
            % Get logical matrix from positions based on tolerance
            rows = ismembertol(detected, ws.Center, 0.1, 'ByRows', true);
            if ~rows
                % If no existing detections are within tolerance, add it
                detected = [detected; ws.Center];
                axes(hTopViewAxes);
                plot(ws);
            
                axes(hChaseViewAxes);
            plot(cuboidModel([ws.Center+[0,0,2], 0.2, 0.2, 1, 0, 0, 0]));
            else
                % If an existing detection is within tolerance, update it
                disp(detected);
                disp(rows);
                disp(detected(rows, :));
                detected(rows, :) = ws.Center;
            end
        end
        
        objectsStruct = [objects{:}];
    
        % Check for camera detections
        if ~isempty(objectsStruct)
            for i = 1 : length(objectsStruct)
                % Get detection location relative to closest vehicle
                o = objectsStruct(i);
                dx = closest.Center(1) - o.Measurement(1);
                dy = closest.Center(2) - o.Measurement(2);
                % If the camera point is within the vehicle's radius
                if(dx^2+dy^2 <= ((closest.Dimensions(1)/2)^2)*1.15)
                    % Get plate number (actor name)
                    actor = scenario.Actors(o.ObjectAttributes{1,1}.TargetIndex);
                    % If the plate hasn't been seen before, add it to array and database
                    if ~any(strcmp(plates, actor.Name))
                        plates = [plates actor.Name];
                        disp(plates);
                        
                        % Prepare and upload to database
                        ws = LidarLib.cuboid2Inertial(closest, vehicle);
                        data = table(ws.Center(1),ws.Center(2),actor.Name,'VariableNames', {'x', 'y', 'plate'});
                        sqlwrite(conn, 'vehicles', data);
                    end
                end
            end
        end
   end
        
    % Update message
    s = size(detected);
    message = sprintf('Number of vehicles detected: %d\nPlates detected: %s', s(1), join(["" plates], ","));
    textField.String = message;
end

function [res] = onFilter(model, inertial)
    global closest;
    res = false;
    if isempty(closest)
        % If there isn’t a closest vehicle then this closest
        closest = model;
    elseif model.Center(1) < closest.Center(1)
        % If the vehicle has a smaller forward distance in vehicle space then it is the closest
        closest = model;
    end
end

