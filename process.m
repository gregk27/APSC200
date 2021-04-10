function [] = process(scenario, objects, ptCloud, vehicle)
    persistent detected;
    persistent textField;
    persistent hTopViewAxes;
    persistent hChaseViewAxes;
    persistent conn;
    global closest;
    global plates;
    if isempty(conn)
        conn = dbconn();
    end
    if isempty(textField)
        detected = [];
        plates = [];
        [textField, hTopViewAxes, hChaseViewAxes] = plotScenario(scenario, vehicle);
                
        % Mark areas that are ticketable 
        areas = select(conn, "SELECT x0,y0,x1,y1 FROM areas");
        areaCount = size(areas);
        areaCount = areaCount(1);
                
        for rowIdx = 1:areaCount
            x0 = double(areas{rowIdx, 1});
            y0 = double(areas{rowIdx, 2});
            x1 = double(areas{rowIdx, 3});
            y1 = double(areas{rowIdx, 4});
            axes(hTopViewAxes);
            plot(cuboidModel([x0-abs(x1-x0)/2, y0-abs(y1-y0)/2, -0.5, abs(x1-x0), abs(y1-y0), 0.9, 0, 0, 0]));
            axes(hChaseViewAxes);
            plot(cuboidModel([x0-abs(x1-x0)/2, y0-abs(y1-y0)/2, -0.5, abs(x1-x0), abs(y1-y0), 0.9, 0, 0, 0]));
        end  
    end

    % plot3(allPosInertial(1,:), allPosInertial(2,:), allPosInertial(3,:), 'b. ', 'Parent', hTopViewAxes);
    
    closest = [];

    [cuboids, cloud, fig] = LidarLib.process(ptCloud, scenario, vehicle, 'minSize', 5, 'maxSize', 35, 'minX', 0, 'maxX', 12.5, 'maxY', -0.5, 'minY', -3,...
        'minYaw', -60, 'maxYaw', 60, 'minRatio', 1, 'maxRatio', 3, 'plot', 'filtered', 'callback', @onFilter, 'roi', [-1, 30, -10, 0.5, 0, 5]);
    
    inZone = false;
    
    if ~isempty(closest)
        % Get the areas that are ticketable 
        areas = select(conn, "SELECT x0,y0,x1,y1 FROM areas");
        areaCount = size(areas);
        areaCount = areaCount(1);
        
        % Get vehicle position in interial (world) space
        pos = LidarLib.cuboid2Inertial(closest, vehicle).Center;
        
        for rowIdx = 1:areaCount
            x0 = double(areas{rowIdx, 1});
            y0 = double(areas{rowIdx, 2});
            x1 = double(areas{rowIdx, 3});
            y1 = double(areas{rowIdx, 4});
            if inpolygon(pos(1), pos(2), [x0, x1, x1, x0], [y0, y0, y1, y1])
                inZone = true;
                break;
            end
        end    
    end
        
    if ~isempty(closest) && inZone
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
            rows = ismembertol(detected, ws.Center, 0.12, 'ByRows', true);
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

