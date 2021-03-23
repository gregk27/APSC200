function [cuboid] = cuboid2Inertial(cuboid, egoVehicle)
    % Derived from vehicle2Inertial

    params = cuboid.Parameters;
    positions = params(1:3).';

    % Euler angles defining orientation of local axes
    yaw = -egoVehicle.Yaw;
    pitch = egoVehicle.Pitch;
    roll =  egoVehicle.Roll;

    % Create orientation matrix from Euler angles using quaternion class
    q = quaternion([yaw pitch roll],'eulerd','zyx','frame');
    myRotationMatrix = rotmat(q,'frame');

    posInertial = myRotationMatrix*positions + egoVehicle.Position';
    
    % Shift angles based on vehicle orientation
    angles = params(7:9) + [pitch roll -yaw];    

    params = [ posInertial.' params(4:6) angles ];
    cuboid = cuboidModel(params);
end