function [pts, image] = take_picture (vrep, id, h, angl, view_angl)

    % Set the rotation
    vrep.simxSetObjectOrientation(id, h.rgbdCasing, h.ref,...
            [0 0 angl], vrep.simx_opmode_oneshot);
    
    % Reduce the view angle to better see the objects
    res = vrep.simxSetFloatSignal(id, 'rgbd_sensor_scan_angle', view_angl,...
        vrep.simx_opmode_oneshot_wait);
    vrchk(vrep, res);
    
    % Ask the sensor to turn itself on, take A SINGLE 3D IMAGE,
    % and turn itself off again
    res = vrep.simxSetIntegerSignal(id, 'handle_xyz_sensor', 1,...
        vrep.simx_opmode_oneshot_wait);
    vrchk(vrep, res);
    fprintf('Capturing point cloud...\n');
    pts = youbot_xyz_sensor(vrep, h, vrep.simx_opmode_oneshot_wait);
    
    pts = pts(1:3,pts(4,:) < 1.5);
    
    % Read data from the RGB camera
    res = vrep.simxSetIntegerSignal(id, 'handle_rgb_sensor', 1,...
        vrep.simx_opmode_oneshot_wait);
    vrchk(vrep, res);
    fprintf('Capturing image...\n');
    [res resolution image] = ...
        vrep.simxGetVisionSensorImage2(id, h.rgbSensor, 0,...
        vrep.simx_opmode_oneshot_wait);
    vrchk(vrep, res);
    fprintf('Captured %i pixels.\n', resolution(1)*resolution(2));
    
    % vrep.simxSetObjectOrientation(id, h.rgbdCasing, h.ref, [0 0 -pi/2], vrep.simx_opmode_oneshot);
end