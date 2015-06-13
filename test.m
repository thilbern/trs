    elseif strcmp(fsm, 'move_arm'),
        res = vrep.simxSetIntegerSignal(id, 'km_mode', 2,...
                vrep.simx_opmode_oneshot_wait);
        
        [res tpos] = vrep.simxGetObjectPosition(id, h.ptip, h.armRef,...
            vrep.simx_opmode_buffer);
        vrchk(vrep, res, true);
        
        tpos = [0.0673 -0.2617 0.2018]
        res = vrep.simxSetObjectPosition(id, h.ptarget, h.armRef, tpos,...
            vrep.simx_opmode_oneshot);
        vrchk(vrep, res, true);
        
        fsm ='extend';
        
    elseif strcmp(fsm, 'extend'),
        [res tpos] = vrep.simxGetObjectPosition(id, h.ptip, h.armRef,...
            vrep.simx_opmode_buffer);
        vrchk(vrep, res, true);
        if norm(tpos-[0.0673 -0.2617 0.2018]) < .002,
            res = vrep.simxSetIntegerSignal(id, 'km_mode', 2,...
                vrep.simx_opmode_oneshot_wait);
            fsm = 'reachout';
        end
    elseif strcmp(fsm, 'reachout'),
        [res tpos] = vrep.simxGetObjectPosition(id, h.ptip, h.armRef,...
            vrep.simx_opmode_buffer);
        vrchk(vrep, res, true);
        
        if tpos(1) > .39,
            fsm = 'grasp';
        end
        
        tpos(1) = tpos(1)+.01;
        res = vrep.simxSetObjectPosition(id, h.ptarget, h.armRef, tpos,...
            vrep.simx_opmode_oneshot);
        vrchk(vrep, res, true);
    elseif strcmp(fsm, 'grasp'),
        res = vrep.simxSetIntegerSignal(id, 'gripper_open', 0,...
            vrep.simx_opmode_oneshot_wait);
        vrchk(vrep, res);
        pause(2);
        res = vrep.simxSetIntegerSignal(id, 'km_mode', 0,...
            vrep.simx_opmode_oneshot_wait);
        fsm = 'backoff';
    elseif strcmp(fsm, 'backoff'),
        for i = 1:5,
            res = vrep.simxSetJointTargetPosition(id, h.armJoints(i),...
                startingJoints(i),...
                vrep.simx_opmode_oneshot);
            vrchk(vrep, res, true);
        end
        [res tpos] = vrep.simxGetObjectPosition(id, h.ptip, h.armRef,...
            vrep.simx_opmode_buffer);
        vrchk(vrep, res, true);
        if norm(tpos-homeGripperPosition) < .02,
            res = vrep.simxSetIntegerSignal(id, 'gripper_open', 1,...
                vrep.simx_opmode_oneshot_wait);
            vrchk(vrep, res);
        end
        if norm(tpos-homeGripperPosition) < .002,
            fsm = 'finished';
        end
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        function xyztable = find_objects (vrep, id, h, youbotPos, youbotEuler, tables, xyztable)
    [i j] = wrapper_vrep_to_matrix(youbotPos(1), youbotPos(2));
    from = double([i j]);
    deltax = tables(2,1) - from(1);
    deltay = tables(2,2) - from(2);
    angl = atan2(deltay, deltax);
    angl = angdiff(angl, youbotEuler(3));
   
    [pts image] = take_picture(vrep, id, h, (angl-(pi/2)-(pi/16)), pi/4, -0.04);
    
    %size(pts);
    %[idx, C] = kmeans(pts,5);
    
    % Plot 3D points and save them to file
    subplot(324)
    cla
    plot3(pts(1,:), pts(2,:), pts(3,:), '*'); % C(1,:), C(2,:), C(3,:), '*');
    axis equal;
    view([-169 -46]);
    fileID = fopen('pc.xyz','w');
    fprintf(fileID,'%f %f %f\n',pts);
    fclose(fileID);
    fprintf('Read %i 3D points, saved to pc.xyz.\n', max(size(pts)));
   
        
    % Change axis of xyz points
    [res camPos] = vrep.simxGetObjectPosition(h.id, h.rgbdCasing, h.ref, vrep.simx_opmode_oneshot_wait); 
    vrchk(vrep, res);
    [res camOri] = vrep.simxGetObjectOrientation(h.id, h.rgbdCasing, h.ref,  vrep.simx_opmode_oneshot_wait); 
    vrchk(vrep, res);
    
    % Initialyze the transformation matrix 
    teta = youbotEuler(3) - camOri(3) -pi/2 ;
    radtodeg(teta)
    dx = camPos(1)
    dy = camPos(2)
    transf = [cos(teta) -sin(teta) dx ; sin(teta) cos(teta) dy ; 0 0 1];
    
    size(xyztable)
    size(transf * pts)
    trans =  [pts(1,:) + dx ; pts(2,:) ; pts(3,:) + dy];
    xyztable = horzcat(xyztable,  trans);
    subplot(325)
    plot3(xyztable(1,:), xyztable(2,:), xyztable(3,:), '*'); % C(1,:), C(2,:), C(3,:), '*');
    axis equal;
    
    
    
    
    
    
    
    
    
    % % Display the RGB image
    % subplot(325)
    % imshow(image);
    % drawnow;
end















    % Initialyze the transformation matrix YOUBOT to world
    teta = youbotEuler(3);
    dx = youbotPos(1);
    dy = youbotPos(2);
    transf2 = [cos(teta) -sin(teta) dx ; sin(teta) cos(teta) dy ; 0 0 1];