function youbot()
% youbot Illustrates the V-REP Matlab bindings.

% (C) Copyright Renaud Detry 2013.
% Distributed under the GNU General Public License.
% (See http://www.gnu.org/copyleft/gpl.html)

disp('Program started');
%Use the following line if you had to recompile remoteApi
%vrep = remApi('remoteApi', 'extApi.h');
vrep=remApi('remoteApi');
vrep.simxFinish(-1);
id = vrep.simxStart('127.0.0.1', 19997, true, true, 2000, 5);

if id < 0,
    disp('Failed connecting to remote API server. Exiting.');
    vrep.delete();
    return;
end
fprintf('Connection %d to remote API server open.\n', id);

% Make sure we close the connexion whenever the script is interrupted.
cleanupObj = onCleanup(@() cleanup_vrep(vrep, id));

% This will only work in "continuous remote API server service"
% See http://www.v-rep.eu/helpFiles/en/remoteApiServerSide.htm
res = vrep.simxStartSimulation(id, vrep.simx_opmode_oneshot_wait);
% We're not checking the error code - if vrep is not run in continuous remote
% mode, simxStartSimulation could return an error.
% vrchk(vrep, res);

% Retrieve all handles, and stream arm and wheel joints, the robot's pose,
% the Hokuyo, and the arm tip pose.
h = youbot_init(vrep, id);
h = youbot_hokuyo_init(vrep, h);

% Let a few cycles pass to make sure there's a value waiting for us next time
% we try to get a joint angle or the robot pose with the simx_opmode_buffer
% option.
pause(.2);

% Constants:

timestep = .05;
wheelradius = 0.0937/2; % This value may be inaccurate. Check before using.

% Min max angles for all joints:
armJointRanges = [-2.9496064186096,2.9496064186096;
    -1.5707963705063,1.308996796608;
    -2.2863812446594,2.2863812446594;
    -1.7802357673645,1.7802357673645;
    -1.5707963705063,1.5707963705063 ];

startingJoints = [0,30.91*pi/180,52.42*pi/180,72.68*pi/180,0];

% In this demo, we move the arm to a preset pose:
pickupJoints = [90*pi/180, 19.6*pi/180, 113*pi/180, -41*pi/180, 0*pi/180];

% Tilt of the Rectangle22 box
r22tilt = -44.56/180*pi;


% Parameters for controlling the youBot's wheels:
forwBackVel = 0;
leftRightVel = 0;
rotVel = 0;

disp('Starting robot');

% Set the arm to its starting configuration:
res = vrep.simxPauseCommunication(id, true); vrchk(vrep, res);
for i = 1:5,
    res = vrep.simxSetJointTargetPosition(id, h.armJoints(i),...
        startingJoints(i),...
        vrep.simx_opmode_oneshot);
    vrchk(vrep, res, true);
end
res = vrep.simxPauseCommunication(id, false); vrchk(vrep, res);

plotData = true;
if plotData,
    subplot(211)
    drawnow;
    subplot(212)
    drawnow;
    [X,Y] = meshgrid(-5:.25:5,-5.5:.25:2.5);
    X = reshape(X, 1, []);
    Y = reshape(Y, 1, []);
end

% Initialyze a matrix to represent the map
map = zeros(100, 100);
map2 = zeros(100, 100);

[i j] = wrapper_vrep_to_matrix([-5 -5], [-5.25 -5.5])
robot_path = [i ; j]' % Matrix representation


% Make sure everything is settled before we start
pause(2);

[res homeGripperPosition] = ...
    vrep.simxGetObjectPosition(id, h.ptip,...
    h.armRef,...
    vrep.simx_opmode_buffer);
vrchk(vrep, res, true);
fsm = 'checkpoint';

while true,
    tic
    if vrep.simxGetConnectionId(id) == -1,
        error('Lost connection to remote API.');
    end
    
    % Get the position and the rotation of the robot
    [res youbotPos] = vrep.simxGetObjectPosition(id, h.ref, -1,...
        vrep.simx_opmode_buffer);
    vrchk(vrep, res, true);
    [res youbotEuler] = vrep.simxGetObjectOrientation(id, h.ref, -1,...
        vrep.simx_opmode_buffer);
    vrchk(vrep, res, true);
    
    % Read data from the Hokuyo sensor:
    [pts contacts] = youbot_hokuyo(vrep, h, vrep.simx_opmode_buffer);
        
    in = inpolygon(X, Y, [h.hokuyo1Pos(1) pts(1,:) h.hokuyo2Pos(1)],...
        [h.hokuyo1Pos(2) pts(2,:) h.hokuyo2Pos(2)]);
    
    % Learn the map, initialyze the transformation matrix 
    teta = youbotEuler(3);
    dx = youbotPos(1);
    dy = youbotPos(2);
    transf = [cos(teta) -sin(teta) dx ; sin(teta) cos(teta) dy ; 0 0 1];
    
    % Learn the map, fill the map matrix
    xin = X(in);
    yin = Y(in);
    xwall = pts(1,contacts);
    ywall = pts(2,contacts);
    
    tmp = ones(3, size(xin, 2));
    tmp(1,:) = xin;
    tmp(2,:) = yin;
    tmp = transf * tmp;
    [tmp(1,:) tmp(2,:)] = wrapper_vrep_to_matrix(tmp(1,:), tmp(2,:));
    for m = tmp(1:2,:)
        map(int32(m(1)), int32(m(2))) =  map(int32(m(1)), int32(m(2))) - 1;
    end
 
    tmp = ones(3, size(xwall, 2));
    tmp(1,:) = xwall;
    tmp(2,:) = ywall;
    tmp = transf * tmp;
    [tmp(1,:) tmp(2,:)] = wrapper_vrep_to_matrix(tmp(1,:), tmp(2,:));
    for m = tmp(1:2,:)
        map(int32(m(1)), int32(m(2))) =  map(int32(m(1)), int32(m(2))) + 1;
    end
 
    
    % Print graphics
    if plotData,
        % Print Hokuyo sensos
        subplot(211)
        plot(xin, yin,'.c', pts(1,contacts), pts(2,contacts), '*b', [h.hokuyo1Pos(1) pts(1,:) h.hokuyo2Pos(1)], [h.hokuyo1Pos(2) pts(2,:) h.hokuyo2Pos(2)], 'r', 0, 0, 'ob', h.hokuyo1Pos(1), h.hokuyo1Pos(2), 'or', h.hokuyo2Pos(1), h.hokuyo2Pos(2), 'or');
        axis([-5.5 5.5 -5.5 2.5]);
        axis equal;
        drawnow;
        
        % Print the map
        subplot(212)
        [row_neg, col_neg] = find(map < 0);
        [row_zero, col_zero] = find(map == 0);
        [row_pos, col_pos] = find(map > 0);
    
        [x_neg, y_neg] = wrapper_matrix_to_vrep(row_neg, col_neg);
        [x_zero, y_zero] = wrapper_matrix_to_vrep(row_zero, col_zero);
        [x_pos, y_pos] = wrapper_matrix_to_vrep(row_pos, col_pos);
        
        plot(x_neg, y_neg, '.c', x_zero, y_zero, '.r', x_pos, y_pos, '.b')

        % if size(robot_path, 1) > 0,
        %     plot(x_neg, y_neg, '.c', x_zero, y_zero, '.r', x_pos, y_pos, '.b', robot_path(:,1), robot_path(:,2), '*g');
        % else
        %     plot(x_neg, y_neg, '.c', x_zero, y_zero, '.r', x_pos, y_pos, '.b')
        % end
        
        axis([-8 8 -8 8]);
        axis equal;
        drawnow;
    end
    
    if strcmp(fsm, 'checkpoint'),
        if size(robot_path, 1) > 0,
            to = robot_path(1,:);
            robot_path = robot_path(2:end,:);
            fsm = 'findangle';
        else
            maptmp = map_refactor(map);
            
            izeros = find(maptmp == 0);
            
            map2(find(maptmp < 0)) = 0;
            map2(izeros) = 0;
            map2(find(maptmp > 0)) = 1;
            
            % Get the "goal" point
            [i j] = wrapper_vrep_to_matrix(youbotPos(1), youbotPos(2));
            goal = int32([j i])
            
            %Initialyze the navigation object
            dx = DXform(map2, 'metric', 'cityblock');
            dx.plot()
            pause(5)
            dx.plan(goal);
            dx.plot()
            pause(5)
            
            % Get the "start" point
            tmp = dx.distancemap(izeros);
            i2 = find(tmp == min(tmp));

            if size(i2, 1) > 0,
                [i j] = ind2sub(size(maptmp), izeros(i2(1)));
                start = [j i]
                
                % Find the robot_path to the start point
                dx.path(start);
                robot_path = dx.path(start);
                robot_path = [flipud(robot_path) ; start];
                tmp = robot_path(:,1);
                robot_path(:,1) = robot_path(:,2);
                robot_path(:,2) = tmp;
                robot_path
                pause(5);
            else
                disp('No start point');
                fsm = 'finished';
            end
        end
    
    elseif strcmp(fsm, 'findangle'),
        [i j] = wrapper_vrep_to_matrix(youbotPos(1), youbotPos(2));
        from = double([i j]);
        to = double(to);
        deltax = to(1) - from(1);
        deltay = to(2) - from(2);
        angl = atan2(deltay, deltax);
        fsm = 'rotate';
    
    elseif strcmp(fsm, 'rotate'),
        forwBackVel = 0;
        leftRightVel = 0;
        rotVel = 10*angdiff(angl, youbotEuler(3));
        if abs(angdiff(angl, youbotEuler(3))) < 1/180*pi,
            rotVel = 0;
            fsm = 'drive';
        end

    elseif strcmp(fsm, 'drive'),
        leftRightVel = 0;
        rotVel = 0;
        [i j] = wrapper_vrep_to_matrix(youbotPos(1), youbotPos(2));
        d = pdist([to ; [i j]], 'euclidean');
        forwBackVel = -20*d;
        
        if (abs(angdiff(angl, youbotEuler(3))) > 3/180*pi) && (d > 1),
            fsm = 'findangle';
        else
            fsm = 'findangle';
        end
        
        if d < .5,
            forwBackVel = 0;
            fsm = 'checkpoint';
        end
        
    elseif strcmp(fsm, 'finished'),
        pause(3);
        break;
    else
        error(sprintf('Unknown state %s.', fsm));
    end
    
    % Update wheel velocities
    res = vrep.simxPauseCommunication(id, true); vrchk(vrep, res);
    vrep.simxSetJointTargetVelocity(id, h.wheelJoints(1),...
        -forwBackVel-leftRightVel+rotVel,...
        vrep.simx_opmode_oneshot); vrchk(vrep, res);
    vrep.simxSetJointTargetVelocity(id, h.wheelJoints(2),...
        -forwBackVel+leftRightVel+rotVel,...
        vrep.simx_opmode_oneshot); vrchk(vrep, res);
    vrep.simxSetJointTargetVelocity(id, h.wheelJoints(3),...
        -forwBackVel-leftRightVel-rotVel,...
        vrep.simx_opmode_oneshot); vrchk(vrep, res);
    vrep.simxSetJointTargetVelocity(id, h.wheelJoints(4),...
        -forwBackVel+leftRightVel-rotVel,...
        vrep.simx_opmode_oneshot); vrchk(vrep, res);
    res = vrep.simxPauseCommunication(id, false); vrchk(vrep, res);
    
    % Make sure that we do not go faster that the simulator
    elapsed = toc;
    timeleft = timestep-elapsed;
    if (timeleft > 0),
        pause(min(timeleft, .01));
    end
end

end % main function

