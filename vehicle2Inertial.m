function [posInertial] = vehicle2Inertial(objectDetections, egoVehicle)
% Helper function for APSC 200 project.
% This function converts vehicle coordenates to inertial coordinates.
% Input: objectDetections containing all object detections, and egoVehicle
% Output: posInertial containing the position of the measurements with
% respect to the inertial frame

objectsStruct =[objectDetections{:}];
allMeasurements = [objectsStruct.Measurement];
positions = allMeasurements(1:3,:);

% Euler angles defining orientation of local axes
yaw = -egoVehicle.Yaw;
pitch = egoVehicle.Pitch;
roll =  egoVehicle.Roll;

% Create orientation matrix from Euler angles using quaternion class
q = quaternion([yaw pitch roll],'eulerd','zyx','frame');
myRotationMatrix = rotmat(q,'frame');

posInertial = myRotationMatrix*positions + egoVehicle.Position';
end

