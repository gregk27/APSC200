function [] = process(scenario, objects, ptCloud, vehicle)
    persistent detected;
    persistent textField;
    persistent hTopViewAxes;
    global closest;
    if isempty(textField)
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
    
    closest = [];

    [cuboids, cloud, fig] = LidarLib.process(ptCloud, scenario, vehicle, 'minSize', 5, 'minX', 0, 'maxY', -0.5, 'minY', -3,...
        'plot', 'filtered', 'callback', @onFilter, 'roi', [-1, 20, -10, 0.5, 0, 3]);

    
    if ~isempty(closest)
        figure(fig);
        plot(closest);
        ws = LidarLib.cuboid2Inertial(closest, vehicle);
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

function [res] = onFilter(model, inertial)
    global closest;
    res = false;
    if isempty(closest)
        closest = model;
    elseif model.Center(1) < closest.Center(1)
        closest = model;
    end
end

