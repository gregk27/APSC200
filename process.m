function [] = process(scenario, objects, ptCloud, vehicle)
    persistent detected;
    persistent textField;
    persistent hTopViewAxes;
    persistent hChaseViewAxes;
    global closest;
    global plates;
    if isempty(textField)
        detected = [];
        plates = [];
        [textField, hTopViewAxes, hChaseViewAxes] = plotScenario(scenario, vehicle);
    end

    % plot3(allPosInertial(1,:), allPosInertial(2,:), allPosInertial(3,:), 'b. ', 'Parent', hTopViewAxes);
    
    closest = [];

    [cuboids, cloud, fig] = LidarLib.process(ptCloud, scenario, vehicle, 'minSize', 5, 'maxSize', 35, 'minX', 0, 'maxX', 12.5, 'maxY', -0.5, 'minY', -3,...
        'minRatio', 1, 'maxRatio', 3, 'plot', 'filtered', 'callback', @onFilter, 'roi', [-1, 30, -10, 0.5, 0, 5]);

    if ~isempty(closest)
        figure(fig);
        plot(closest);
        ws = LidarLib.cuboid2Inertial(closest, vehicle);
        if isempty(detected)
            detected = [ws.Center];
            axes(hTopViewAxes);
            plot(ws);
            
            axes(hChaseViewAxes);
            plot(cuboidModel([ws.Center+[0,0,2], 0.2, 0.2, 1, 0, 0, 0]));
        else
            rows = ismembertol(detected, ws.Center, 0.1, 'ByRows', true);
            if ~rows
                detected = [detected; ws.Center];
                axes(hTopViewAxes);
                plot(ws);
            
                axes(hChaseViewAxes);
            plot(cuboidModel([ws.Center+[0,0,2], 0.2, 0.2, 1, 0, 0, 0]));
            else
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
                o = objectsStruct(i);
                dx = closest.Center(1) - o.Measurement(1);
                dy = closest.Center(2) - o.Measurement(2);
                % If the camera point is within the vehicle's radius
                if(dx^2+dy^2 <= ((closest.Dimensions(1)/2)^2)*1.15)
                    actor = scenario.Actors(o.ObjectAttributes{1,1}.TargetIndex);
                    if ~any(strcmp(plates, actor.Name))
                        plates = [plates actor.Name];
                        disp(plates);
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
        closest = model;
    elseif model.Center(1) < closest.Center(1)
        closest = model;
    end
end

