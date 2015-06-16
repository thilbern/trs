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
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
objectid = 1;       
xyztable(1:3,find(idx==objectid)))
m = C(:,objectid)'








% TLS Line
% (X-P(1))/N(1) = (Y-P(2))/N(2) = (Z-P(3))/N(3)
% P is a point on the fitted line
% and N its direction vector.

% test data
Data = importdata('youbot/pc.xyz');
Data = Data';

% Compute KMEANS
[idx, C] = kmeans15(Data', 5, 'maxIter', 200, 'replicates', 10);
C = C';
plot3(X(1,:), X(2,:), X(3,:), '*b', C(1,:), C(2,:), C(3,:), 'or');
hold on

for id = 1:1:size(C,2),
    d = Data(1:3,find(idx==id));
    x = d(1,:)';
    y = d(3,:)';
    z = d(3,:)';
    n = size(x,1);
    
    % line fit
    P=[mean(x),mean(y),mean(z)]';
    [U,S,V]=svd([x-P(1),y-P(2),z-P(3)]);
    N=1/V(end,1)*V(:,1);

    % Plot
    A=P+dot([x(1),y(1),z(1)]'-P,N)*N/norm(N)^2;
    B=P+dot([x(n),y(n),z(n)]'-P,N)*N/norm(N)^2;
    plot3([A(1),B(1)],[A(2),B(2)],[A(3),B(3)])
end
grid; hold off