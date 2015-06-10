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