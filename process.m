function [] = process(scenario, objects, ptCloud, vehicle)
    persistent detected;
    persistent textField;
    persistent hTopViewAxes;
    persistent hChaseViewAxes;
    global closest;
    if isempty(textField)
        detected = [];
        [textField, hTopViewAxes, hChaseViewAxes] = plotScenario(scenario, vehicle);
    end

    % objectsStruct = [objects{:}];
    
     %if ~isempty(objects)
     %        allPosInertial = vehicle2Inertial(objects, vehicle);
     %        disp('Class ids: %i\n');
     %        disp([objectsStruct.ObjectClassID]);

     %end

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
   end
        
    % Update message
    s = size(detected);
    message = sprintf('Number of vehicles detected: %d\n', s(1));
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

