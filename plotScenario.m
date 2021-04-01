function [textField, hTopViewAxes, hChaseViewAxes] = plotScenario(scenario,egoVehicle)
% Helper function for APSC 200 project.
% This function creates the visalization interface
% Input: scenario, egoVehicle
% Both input parameters come from the exported scenario file
% Output: textField for you to be able to change the text in the interface
%         hTopViewAxes for you to be able to plot new points if you want



figscenario = findall(0, 'Type', 'Figure', 'Tag', 'VIODemoDisplay');
if isempty(figscenario)
    figscenario = figure('Name', 'Driving Scenario', ...
        'Tag', 'VIODemoDisplay', 'Position', [0, 0, 1032, 600]);
    movegui(figscenario, [0 -1]);
else
    figure(figscenario);
end
clf(figscenario);

% Plot the top view of the scenario.
hTopViewPanel = uipanel(figscenario, 'Position', [0 0.12 0.5 1], ...
    'Title', 'Bird''s Eye View');
hTopViewAxes = axes(hTopViewPanel);
plot(scenario, 'Parent', hTopViewAxes);
hold on
% xlim(hTopViewAxes,[-20 350])
% ylim(hTopViewAxes,[-50 350])
% zlim(hTopViewAxes,[0 200])
view(hTopViewAxes,-85,60)

% Plot the chase view of the scenario from the rear of the vehicle.
hChaseViewPanel = uipanel(figscenario, 'Position', [0.5 0.12 0.5 1], ...
    'Title', 'Chase View');
hChaseViewAxes = axes(hChaseViewPanel);
chasePlot(egoVehicle, 'Parent', hChaseViewAxes);
% xlim(hChaseViewAxes,[-20 350])
% ylim(hChaseViewAxes,[-50 350])
% zlim(hChaseViewAxes,[0 200])


htextPanel = uipanel(figscenario, 'Position', [0 0 1 0.12], ...
    'Title', 'Object detection');

str = 'Text';
textField = uicontrol(htextPanel, 'Style','text', 'String', str,'Position', [10 0 1000 50]);
textField.FontSize = 14;
%textField.BackgroundColor = [1 1 1];
textField.HorizontalAlignment = 'left';
drawnow;
end

