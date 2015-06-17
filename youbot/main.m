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
transportJoints = [0,40*pi/180,52.42*pi/180,62*pi/180,0];

% In this demo, we move the arm to a preset pose:
pickupJoints = [90*pi/180, 19.6*pi/180, 113*pi/180, -41*pi/180, 0*pi/180];

% Tilt of the Rectangle22 box
r22tilt = -44.56/180*pi;


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



% Make sure everything is settled before we start
pause(2);

[res homeGripperPosition] = ...
    vrep.simxGetObjectPosition(id, h.ptip,...
    h.armRef,...
    vrep.simx_opmode_buffer);
vrchk(vrep, res, true);


% Initialyze plotting
plotData = true;
if plotData,
    % close all;
    subplot(321)
    drawnow;
    subplot(322)
    drawnow;
    subplot(323)
    drawnow;
    subplot(324)
    drawnow;
    subplot(325)
    drawnow;
    [X,Y] = meshgrid(-5:.25:5,-5.5:.25:2.5);
    X = reshape(X, 1, []);
    Y = reshape(Y, 1, []);
end

% Extra variables
maptmp = zeros(70, 70);
map2 = zeros(70, 70);
basketsid = 1;
tablesid = 1;
objectsid = 1;
speed_coef = 1;
tpos_arm = [];
alpha = [];

% Map, tables, baskets and objects
map = zeros(70, 70);
baskets_mtrx = [];
baskets_entry_mtrx = [];
tables_mtrx = [0 0 ; 0 0];
tables_vrep = [0 0 ; 0 0];
xyztable = [];
objects_vrep = [];

% Initialyze a first robot path
[i j] = wrapper_vrep_to_matrix([-2 -2], [-4.75 -5.25]);
robot_path = [i ; j]'; % Matrix representation
destination = [];
to = [];
dest_euler = NaN;
prev_dist = NaN;

% Parameters for controlling the youBot's wheels:
forwBackVel = 0;
leftRightVel = 0;
rotVel = 0;

% Determine de milestone
fsm = 'checkpoint';
master_fsm = 'map_discovery';
if exist('map.mat', 'file') == 2 & exist('map.mat', 'file') == 2,
    load('map.mat', '-ascii', 'map')
    load('map2.mat', '-ascii', 'map2')
    master_fsm = 'back_to_origin';
    fsm = 'on_destination';
end

% Print initial value of youbotPos and youbotEuler
[res startYoubotPos] = vrep.simxGetObjectPosition(id, h.ref, -1,...
    vrep.simx_opmode_buffer);
vrchk(vrep, res, true);
[res startYoubotEuler] = vrep.simxGetObjectOrientation(id, h.ref, -1,...
    vrep.simx_opmode_buffer);
vrchk(vrep, res, true);
startYoubotPos
startYoubotEuler
    
% Initialyze main loop
frameID = 0;
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
    [pts contacts closest1 closest2] = youbot_hokuyo(vrep, h, vrep.simx_opmode_buffer);
    
    in = inpolygon(X, Y, [h.hokuyo1Pos(1) pts(1,:) h.hokuyo2Pos(1)],...
        [h.hokuyo1Pos(2) pts(2,:) h.hokuyo2Pos(2)]);
    xin = X(in);
    yin = Y(in);
    
    % Learn the map
    if strcmp(master_fsm, 'map_discovery'),
        map = map_update(map, youbotEuler, youbotPos, pts, contacts, xin, yin);
    end

    % Print graphics
    if plotData,
        % Print Hokuyo sensos
        subplot(321)
        plot(xin, yin,'.c', pts(1,contacts), pts(2,contacts), '*b', [h.hokuyo1Pos(1) pts(1,:) h.hokuyo2Pos(1)], [h.hokuyo1Pos(2) pts(2,:) h.hokuyo2Pos(2)], 'r', 0, 0, 'ob', h.hokuyo1Pos(1), h.hokuyo1Pos(2), 'or', h.hokuyo2Pos(1), h.hokuyo2Pos(2), 'or', closest1(1), closest1(2), '*r', closest2(1), closest2(2), '*r');
        axis([-5.5 5.5 -5.5 2.5]);
        axis equal;
        drawnow;
        
        % Print the map
        map_print(322, map, [to;robot_path]);
        
        % Print the map
        if strcmp(master_fsm, 'map_discovery'),
            map_print(323, maptmp, []);
        else
            map_print(323, map2, [tables_mtrx; baskets_entry_mtrx ; baskets_mtrx]);
        end
    end
    
    if strcmp(fsm, 'on_destination'),
        if strcmp(master_fsm, 'map_discovery'),
            % Get the "goal" point
            [i j] = wrapper_vrep_to_matrix(youbotPos(1), youbotPos(2));
            goal = int32([j i]);
            
            % Refactor the map
            maptmp = map_refactor(map, [i j]);
            
            % Create the map to give to DXform
            izeros = find(map == 0 & maptmp < 0) ;
            map2(find(maptmp < 0)) = 0;
            map2(izeros) = 0;
            map2(find(maptmp > 0)) = 1;
            
            % Initialyze the navigation object
            dx = DXform(map2, 'metric', 'cityblock');
            dx.plan(goal);
            
            % Get the "start" point
            tmp = dx.distancemap(izeros);
            i2 = find(tmp == min(tmp));
            
            if size(i2, 1) > 0,
                [i j] = ind2sub(size(maptmp), izeros(i2(1)));
                start = [j i];
                
                % Find the robot_path to the start point
                robot_path = dx.path(start);
                robot_path = [flipud(robot_path) ; start];
                robot_path = switch_column(robot_path, 1, 2);
                robot_path = reduce_path (robot_path, map2);
                destination = switch_column(start, 1, 2);
                
                fsm = 'checkpoint';
            else
                disp('No start point. The map is complete');
                forwBackVel = 0;
                leftRightVel =0;
                rotVel = 0;
                
                % Save maps
                neg = find(map < 0);
                map = ones(70, 70);
                map(neg) = 0;
                
                map_print(322, map, []);
                map_print(323, map2, []);
                save('map.mat', 'map', '-ascii');
                save('map2.mat', 'map2', '-ascii');
                
                master_fsm = 'back_to_origin';
                fsm = 'on_destination';
            end
            
        elseif strcmp(master_fsm, 'back_to_origin'),
            disp('Back to origin');
            [i j] = wrapper_vrep_to_matrix(youbotPos(1), youbotPos(2));
            from = double([i j]);
            [i j] = wrapper_vrep_to_matrix(startYoubotPos(1), startYoubotPos(2));
            to = double([i j]);
            
            if (pdist([from ; to], 'euclidean') > 2),
                robot_path = compute_path(map, int32(from), int32(to), 1);
                master_fsm = 'back_to_origin';
                fsm = 'checkpoint';
            else
                master_fsm = 'find_tables_basckets';
                fsm = 'on_destination';
            end
            
        elseif strcmp(master_fsm, 'find_tables_basckets'),
            disp('find_tables_basckets');
            [tables_mtrx baskets_mtrx baskets_entry_mtrx] = find_basket (map);
            map_print(323, map, [tables_mtrx; baskets_entry_mtrx ; baskets_mtrx]);
            
            if exist('pc.xyz', 'file') == 2,
                xyztable = importdata('pc.xyz');
                xyztable = xyztable';
                master_fsm = 'discover_tables_objects';
                fsm = 'on_destination';
            else
                master_fsm = 'discover_tables';
                fsm = 'on_destination';
                tablesid = 1;
            end
            
            
        elseif strcmp(master_fsm, 'discover_tables'),
            disp('discover_tables');
            
            if tablesid > 1,
                xyztable = find_objects(vrep, id, h, youbotPos, youbotEuler, tables_mtrx, xyztable);
                if tablesid == 2,
                    disp('RGB photo of the table');
                    view_angl = pi/3;
                    angl = compute_photo_angle(youbotPos, youbotEuler, tables_mtrx(2,:));
                    [pts, image] = take_picture (vrep, id, h, angl-(pi/2), view_angl);
                    
                    % Initialyze the transformation matrix youBot to Vrep
                    transf = [cos(youbotEuler(3)) -sin(youbotEuler(3)) youbotPos(1) ; sin(youbotEuler(3)) cos(youbotEuler(3)) youbotPos(2) ; 0 0 1];
        
                    % Position of camera in VREP
                    camVrep = transf * [0 ; -0.25 ; 1];
                    camVrep(3) = 0.2257;
                    camVrep
                    angl
                    view_angl
                    
                    % Display the RGB image
                    subplot(325)
                    imshow(image);
                    imsave
                    drawnow;
                end
            end
            if tablesid < 9,
                r = 3;
                if tablesid == 1,
                    dest = tables_mtrx(2,:) + [-r 0];
                    dest_euler = compute_angle(dest, tables_mtrx(2,:)) - pi/2;
                    
                elseif tablesid == 2,
                    dest = tables_mtrx(2,:) + [-r -r];
                    dest_euler = compute_angle(dest, tables_mtrx(2,:)) - pi/2;
                    
                elseif tablesid == 3,
                    dest = tables_mtrx(2,:) + [0 -r];
                    dest_euler = compute_angle(dest, tables_mtrx(2,:)) - pi/2;
                    
                elseif tablesid == 4,
                    dest = tables_mtrx(2,:) + [r -r];
                    dest_euler = compute_angle(dest, tables_mtrx(2,:)) - pi/2;
                    
                elseif tablesid == 5,
                    dest = tables_mtrx(2,:) + [r 0];
                    dest_euler = compute_angle(dest, tables_mtrx(2,:)) - pi/2;
                    
                elseif tablesid == 6,
                    dest = tables_mtrx(2,:) + [r r];
                    dest_euler = compute_angle(dest, tables_mtrx(2,:)) - pi/2;
                    
                elseif tablesid == 7,
                    dest = tables_mtrx(2,:) + [0 r];
                    dest_euler = compute_angle(dest, tables_mtrx(2,:)) - pi/2;
                    
                elseif tablesid == 8,
                    dest = tables_mtrx(2,:) + [-r r];
                    dest_euler = compute_angle(dest, tables_mtrx(2,:)) - pi/2;
                end
                robot_path = dest;
                speed_coef = 0.2;
                fsm = 'checkpoint';
            else
                % Save the xyz points
                fileID = fopen('pc.xyz','w');
                fprintf(fileID,'%f %f %f\n',xyztable);
                fclose(fileID);
                fprintf('Read %i 3D points, saved to pc.xyz.\n', max(size(xyztable)));
    
                master_fsm = 'discover_tables_objects';
                fsm = 'on_destination';
                speed_coef = 1;
            end
            tablesid = tablesid + 1;


        elseif strcmp(master_fsm, 'discover_tables_objects'),
            disp('discover_tables_objects');
            
            pts_small = xyztable(:,xyztable(3, :) < 0.180);
            pts_hight = xyztable(:,xyztable(3, :) > 0.185);
        
            % Detect table with better precision
            pts_small = pts_small(:,pts_small(3, :) > 0.16);
            P = mean(pts_small');
            tables_vrep(2,:) = P(1,1:2);

            % Compute clustering to dectect objects
            try
                [idx,C] = kmeans15(pts_hight', 5, 'maxIter','100', 'replicates', 15);
            catch ME
                disp('Empty cluster, retry...');
                [idx,C] = kmeans15(pts_hight', 5, 'maxIter','100', 'replicates', 15);
            end
            subplot(324)
            cla
            plot3(xyztable(1,:), xyztable(2,:), xyztable(3,:), '*b', C(:,1), C(:,2), C(:,3), 'or', P(1), P(2), P(3), 'oc');
            axis equal;
            
            objects_vrep = C
            
            master_fsm = 'discover_basckets';
            fsm = 'on_destination';
            basketsid = 0;
        
        
        elseif strcmp(master_fsm, 'discover_basckets'),
            disp('discover_basckets');
            if basketsid ~= 0,

                angl = compute_photo_angle(youbotPos, youbotEuler, baskets_mtrx(basketsid,:));
                [pts, image] = take_picture (vrep, id, h, angl-(pi/2), pi/3);
                
                % Display the RGB image
                subplot(325)
                imshow(image);
                % imsave
                drawnow;
            end
            
            basketsid = basketsid + 1;
            if basketsid <= size(baskets_entry_mtrx, 1),
                [i j] = wrapper_vrep_to_matrix(youbotPos(1), youbotPos(2));
                from = int32([i j]);
                robot_path = compute_path(map2, from, baskets_entry_mtrx(basketsid,:), 1);
                dest_euler = compute_angle(baskets_entry_mtrx(basketsid,:), baskets_mtrx(basketsid,:)) - pi/2;
                master_fsm = 'discover_basckets';
                fsm = 'checkpoint';
            else
                fsm = 'on_destination';
                master_fsm = 'back_to_origin2';
                basketsid = 1;
                objectsid = 1;
            end
            master_fsm = 'reach_object';
            fsm = 'on_destination';
            
        elseif strcmp(master_fsm, 'back_to_origin2'),
            disp('Back to origin2');
            if objectsid <= size(objects_vrep, 1)
                [i j] = wrapper_vrep_to_matrix(youbotPos(1), youbotPos(2));
                from = double([i j]);
                [i j] = wrapper_vrep_to_matrix(startYoubotPos(1), startYoubotPos(2));
                to = double([i j]);
                
                if (pdist([from ; to], 'euclidean') > 2),
                    robot_path = compute_path(map, int32(from), int32(to), 1);
                    speed_coef = 1;
                    master_fsm = 'back_to_origin2';
                    fsm = 'checkpoint';
                else
                    master_fsm = 'reach_object';
                    fsm = 'on_destination';
                end
            else
                fsm = 'finished';
            end
            
            
        elseif strcmp(master_fsm, 'reach_object'),
            disp('reach_object');
            [i j] = wrapper_vrep_to_matrix(youbotPos(1), youbotPos(2));
            from = [i j];
            [i j] = wrapper_vrep_to_matrix(objects_vrep(objectsid,1), objects_vrep(objectsid,2));
            target = [i j];
            dest = target-tables_mtrx(2,:);
            dest = dest/norm(dest);

            robot_path = compute_path(map2, int32(from), int32(tables_mtrx(2,:) + dest*5), 0);
            dest = tables_mtrx(2,:) + dest*2.3;
            robot_path = [robot_path ; dest];
            dest_euler = compute_angle(dest, tables_mtrx(2,:)) - pi/2;
            
            res = vrep.simxSetIntegerSignal(id, 'gripper_open', 1,...
                vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);
            
            speed_coef = 0.05;            
            
            fsm = 'checkpoint';
            master_fsm = 'adjust_position';


            
        elseif strcmp(master_fsm, 'adjust_position'),
            disp('adjust_position');
            
            tpos_arm = objects_vrep(objectsid,:);
            tpos_arm = wrapper_vrep_to_arm(youbotPos, youbotEuler, tpos_arm)
            
            forwBackVel = 0;
            leftRightVel = 0;
            rotVel = 0;
            
            if (tpos_arm(2) > .1)
                forwBackVel = 1
            elseif (tpos_arm(2) < -.1)
                forwBackVel = -1
            elseif (tpos_arm(1) > .42)
                leftRightVel = 1
            elseif (tpos_arm(1) < -.42)
                leftRightVel = -1
            end
            
            if (forwBackVel == 0 & leftRightVel == 0 && rotVel == 0)
                alpha = [tpos_arm(1) * .8 ; tpos_arm(2) * .8 ; tpos_arm(3)]
                master_fsm = 'grasp_object';
                fsm = 'on_destination';
            else
                fsm = 'adjust_position';
                fsm = 'on_destination';
            end
            
        elseif strcmp(master_fsm, 'grasp_object'),
            disp('grasp_object');
            fsm = 'move_arm';
            master_fsm = 'reach_basckets',
            
        elseif strcmp(master_fsm, 'reach_basckets'),
            disp('reach_basckets');
            basketsid = 1;
            if basketsid <= size(baskets_entry_mtrx, 1),
                [i j] = wrapper_vrep_to_matrix(youbotPos(1), youbotPos(2));
                from = int32([i j]);
                robot_path = compute_path(map2, from, baskets_entry_mtrx(basketsid,:), 0);
                dest_euler = compute_angle(baskets_entry_mtrx(basketsid,:), baskets_mtrx(basketsid,:)) - pi/2;
                master_fsm = 'adjust_position_basckets';
                fsm = 'checkpoint';
                speed_coef = 1;
            else
                fsm = 'finished';
            end
            
        elseif strcmp(master_fsm, 'adjust_position_basckets'),
            disp('adjust_position_basckets');
            
            tpos_arm = baskets_mtrx(basketsid,:)
            [i j] = wrapper_matrix_to_vrep(tpos_arm(1), tpos_arm(2));
            tpos_arm = wrapper_vrep_to_arm(youbotPos, youbotEuler, [i j .25])
            
            forwBackVel = 0;
            leftRightVel = 0;
            rotVel = 0;
            
            if (tpos_arm(2) > .1)
                forwBackVel = 1
            elseif (tpos_arm(2) < -.1)
                forwBackVel = -1
            elseif (tpos_arm(1) > .42)
                leftRightVel = 1
            elseif (tpos_arm(1) < -.42)
                leftRightVel = -1
            end
            
            if (forwBackVel == 0 & leftRightVel == 0 && rotVel == 0)
                alpha = [tpos_arm(1) * .8 ; tpos_arm(2) * .8 ; tpos_arm(3)]
                master_fsm = 'back_to_origin2';
                fsm = 'move_arm';
                objectsid = objectsid + 1;
            else
                fsm = 'adjust_position_basckets';
                fsm = 'on_destination';
            end
            
        else
            fsm = 'finished';
        end
        
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%% Grasp objects     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    elseif strcmp(fsm, 'move_arm'),
        res = vrep.simxSetIntegerSignal(id, 'km_mode', 1,...
                vrep.simx_opmode_oneshot_wait);
        %tpos_arm already Initialyzed
        % alpha = .8
        % tpos_arm = [-2 -5.25 0.185];
        % tpos_arm = wrapper_vrep_to_arm(youbotPos, youbotEuler, tpos_arm)
        res = vrep.simxSetObjectPosition(id, h.ptarget, h.armRef, alpha,...
            vrep.simx_opmode_oneshot);
        vrchk(vrep, res, true);
        
        fsm ='extend';
        
    elseif strcmp(fsm, 'extend'),
        [res tpos] = vrep.simxGetObjectPosition(id, h.ptip, h.armRef,...
            vrep.simx_opmode_buffer);
        vrchk(vrep, res, true);
        if norm(tpos- alpha') < .002,
            tpos
            res = vrep.simxSetIntegerSignal(id, 'km_mode', 1,...
                vrep.simx_opmode_oneshot_wait);
            alpha = [tpos_arm(1) * .99 ; tpos_arm(2) * .99 ; tpos_arm(3)]
            res = vrep.simxSetObjectPosition(id, h.ptarget, h.armRef, alpha,...
            vrep.simx_opmode_oneshot);
            vrchk(vrep, res, true);
            fsm = 'reachout';
        end
        
    elseif strcmp(fsm, 'reachout'),
        [res tpos] = vrep.simxGetObjectPosition(id, h.ptip, h.armRef,...
            vrep.simx_opmode_buffer);
        vrchk(vrep, res, true);
        if norm(tpos- alpha') < .002,
            tpos
            res = vrep.simxSetIntegerSignal(id, 'km_mode', 1,...
                vrep.simx_opmode_oneshot_wait);
            fsm = 'grasp';
        end
        
    elseif strcmp(fsm, 'grasp'),
        res = vrep.simxSetIntegerSignal(id, 'gripper_open', 1,...
            vrep.simx_opmode_oneshot_wait);
        vrchk(vrep, res);
        pause(2);
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
        % if norm(tpos-homeGripperPosition) < .02,
        %     res = vrep.simxSetIntegerSignal(id, 'gripper_open', 1,...
        %         vrep.simx_opmode_oneshot_wait);
        %     vrchk(vrep, res);
        % end
        if norm(tpos-homeGripperPosition) < .002,
            for i = 1:5,
                res = vrep.simxSetIntegerSignal(id, 'gripper_open', 1,...
                    vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);
                
                res = vrep.simxSetJointTargetPosition(id, h.armJoints(i),...
                    transportJoints(i),...
                    vrep.simx_opmode_oneshot); vrchk(vrep, res, true);

                res = vrep.simxSetIntegerSignal(id, 'gripper_open', 0,...
                    vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);
            end
            fsm = 'on_destination';
        end
       
        
        
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%% Moving to points %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    elseif strcmp(fsm, 'movingtopoint'),
        % Find the cap
        [i j] = wrapper_vrep_to_matrix(youbotPos(1), youbotPos(2));
        from = double([i j]);
        to = double(to);
        angl = compute_angle(from, to);
        
        % find the distance from the destination
        d = pdist([to ; [i j]], 'euclidean');
        
        % de embourbage
        prev_dist = NaN;
        % if (not(isnan(prev_dist))) & (mod(frameID,20) == 0) & (d >= prev_dist),
        tresh = .2 * speed_coef;
        if closest1(4) < tresh | closest2(4) < tresh,
            disp('I m stuck');
            forwBackVel = -forwBackVel;
            rotVel = 0;
            leftRightVel = 0;
            fsm = 'getout';
        else
            if (size(destination, 1) > 0) && (d < 3),
                fsm = 'checkpoint';
            else
                if d < 1,
                    fsm = 'checkpoint';
                else
                    alpha = abs(angdiff(angl, youbotEuler(3)));
                    if alpha < (pi / 16)
                        a = (pi - alpha)/pi*10;
                    elseif alpha < (pi / 8)
                        a = (pi - alpha)/pi*5;
                    else
                        a = (pi - alpha)/pi*1;
                    end
                    forwBackVel = -a*5*d*speed_coef;
                    rotVel = 20 * angdiff(angl, youbotEuler(3));
                    
                    if alpha < (pi / 10)
                        leftRightVel = 2 * angdiff(angl, youbotEuler(3));
                    end
                end
            end
        end
        if (isnan(prev_dist)) |  (mod(frameID,20) == 0),
            prev_dist = d;
        end
        
    elseif strcmp(fsm, 'getout'),
        if (mod(frameID,10) == 0),
            fsm = 'getout_reorientation';
            prev_dist = NaN;
        end
        
    elseif strcmp(fsm, 'getout_reorientation'),
        % Find the cap
        [i j] = wrapper_vrep_to_matrix(youbotPos(1), youbotPos(2));
        from = double([i j]);
        to = double(to);
        angl = compute_angle(from, to);
        
        forwBackVel = 0;
        rotVel = 10*angdiff(angl, youbotEuler(3));
        if abs(angdiff(angl, youbotEuler(3))) < 1/180*pi,
            rotVel = 0;
            fsm = 'movingtopoint';
        end
        
    elseif strcmp(fsm, 'checkpoint'),
        if size(robot_path, 1) > 0,
            to = robot_path(1,:);
            robot_path = robot_path(2:end,:);
            fsm = 'movingtopoint';
        else
            forwBackVel = 0;
            leftRightVel =0;
            rotVel = 0;
            fsm = 'final_rotate';
        end
        
    elseif strcmp(fsm, 'final_rotate'),
        if isnan(dest_euler)
            disp('coucou');
            fsm = 'on_destination';
        else
            rotVel = 10*angdiff(dest_euler, youbotEuler(3));
            if abs(angdiff(dest_euler, youbotEuler(3))) < 1/180*pi,
                rotVel = 0;
                fsm = 'on_destination';
                dest_euler = NaN;
            end
        end
        prev_dist = NaN;
        
        
        
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%% End of simulation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    elseif strcmp(fsm, 'finished'),
        pause(3);
        break;
    else
        error(sprintf('Unknown state %s.', fsm));
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%% Update velicity   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
    frameID = frameID + 1;
end

end % main function

