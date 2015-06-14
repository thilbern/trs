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
    close all;
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
speed_coef = 1;

% Map, tables, baskets and objects
map = zeros(70, 70);
baskets = [];
baskets_entry = [];
tables = [];
xyztable = [];
objects = [];

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
    [pts contacts] = youbot_hokuyo(vrep, h, vrep.simx_opmode_buffer);
    
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
        plot(xin, yin,'.c', pts(1,contacts), pts(2,contacts), '*b', [h.hokuyo1Pos(1) pts(1,:) h.hokuyo2Pos(1)], [h.hokuyo1Pos(2) pts(2,:) h.hokuyo2Pos(2)], 'r', 0, 0, 'ob', h.hokuyo1Pos(1), h.hokuyo1Pos(2), 'or', h.hokuyo2Pos(1), h.hokuyo2Pos(2), 'or');
        axis([-5.5 5.5 -5.5 2.5]);
        axis equal;
        drawnow;
        
        % Print the map
        map_print(322, map, [to;robot_path]);
        
        % Print the map
        if strcmp(master_fsm, 'map_discovery'),
            map_print(323, maptmp, []);
        else
            map_print(323, map2, [tables; baskets_entry ; baskets]);
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
                robot_path = compute_path(map, int32(from), int32(to));
                master_fsm = 'back_to_origin';
                fsm = 'checkpoint';
            else
                master_fsm = 'find_tables_basckets';
                fsm = 'on_destination';
            end
            
        elseif strcmp(master_fsm, 'find_tables_basckets'),
            disp('find_tables_basckets');
            [tables baskets baskets_entry] = find_basket (map);
            map_print(323, map, [tables; baskets_entry ; baskets]);
            
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
                xyztable = find_objects(vrep, id, h, youbotPos, youbotEuler, tables, xyztable);
                if tablesid == 2,
                    disp('RGB photo of the table');
                    view_angl = pi/3;
                    angl = compute_photo_angle(youbotPos, youbotEuler, tables(2,:));
                    [pts, image] = take_picture (vrep, id, h, angl-(pi/2), view_angl, -0.04);
                    
                    % Initialyze the transformation matrix youBot to Vrep
                    transf = [cos(youbotEuler(3)) -sin(youbotEuler(3)) youbotPos(1) ; sin(youbotEuler(3)) cos(youbotEuler(3)) youbotPos(2) ; 0 0 1];
        
                    % Position of camera in VREP
                    camVrep = transf * [0 ; -0.25 ; 1] 
                    angl
                    view_angl
                    
                    % Display the RGB image
                    subplot(325)
                    imshow(image);
                    imsave
                    drawnow;
                    pause(5)
                end
            end
            if tablesid < 9,
                r = 3;
                if tablesid == 1,
                    dest = tables(2,:) + [-r 0];
                    dest_euler = compute_angle(dest, tables(2,:)) - pi/2;
                    
                elseif tablesid == 2,
                    dest = tables(2,:) + [-r -r];
                    dest_euler = compute_angle(dest, tables(2,:)) - pi/2;
                    
                elseif tablesid == 3,
                    dest = tables(2,:) + [0 -r];
                    dest_euler = compute_angle(dest, tables(2,:)) - pi/2;
                    
                elseif tablesid == 4,
                    dest = tables(2,:) + [r -r];
                    dest_euler = compute_angle(dest, tables(2,:)) - pi/2;
                    
                elseif tablesid == 5,
                    dest = tables(2,:) + [r 0];
                    dest_euler = compute_angle(dest, tables(2,:)) - pi/2;
                    
                elseif tablesid == 6,
                    dest = tables(2,:) + [r r];
                    dest_euler = compute_angle(dest, tables(2,:)) - pi/2;
                    
                elseif tablesid == 7,
                    dest = tables(2,:) + [0 r];
                    dest_euler = compute_angle(dest, tables(2,:)) - pi/2;
                    
                elseif tablesid == 8,
                    dest = tables(2,:) + [-r r];
                    dest_euler = compute_angle(dest, tables(2,:)) - pi/2;
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
        
            % Compute clustering to dectect objects
            [idx,C] = kmeans15(xyztable', 5, 'maxIter','100', 'replicates', 15);
            subplot(324)
            cla
            plot3(xyztable(1,:), xyztable(2,:), xyztable(3,:), '*b', C(:,1), C(:,2), C(:,3), 'or');
            axis equal;
            
            objects = C
            
            master_fsm = 'discover_basckets';
            fsm = 'on_destination';
            basketsid = 0;
        
        
        elseif strcmp(master_fsm, 'discover_basckets'),
            disp('discover_basckets');
            if basketsid ~= 0,
            
                angl = compute_photo_angle(youbotPos, youbotEuler, baskets(basketsid,:));
                [pts, image] = take_picture (vrep, id, h, angl-(pi/2), pi/3, -0.04);
                
                % Display the RGB image
                subplot(325)
                imshow(image);
                imsave
                drawnow;
                pause(5)
            end
            
            basketsid = basketsid + 1;
            if basketsid <= size(baskets_entry, 1),
                [i j] = wrapper_vrep_to_matrix(youbotPos(1), youbotPos(2));
                from = int32([i j]);
                baskets_entry(basketsid,:)
                robot_path = compute_path(map2, from, baskets_entry(basketsid,:));
                dest_euler = compute_angle(baskets_entry(basketsid,:), baskets(basketsid,:)) - pi/2;
                master_fsm = 'discover_basckets';
                fsm = 'checkpoint';
            else
                fsm = 'finished';
                basketsid = 1;
            end
            
        else
            fsm = 'finished';
        end
        
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%% Grap objects     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
       
        
        
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%% Moving to points %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    elseif strcmp(fsm, 'movingtopoint'),
        % Find the cap
        [i j] = wrapper_vrep_to_matrix(youbotPos(1), youbotPos(2));
        from = double([i j]);
        to = double(to);
        deltax = to(1) - from(1);
        deltay = to(2) - from(2);
        angl = atan2(deltay, deltax);
        
        % find the distance from the destination
        d = pdist([to ; [i j]], 'euclidean');
        
        % if (mod(frameID,20) == 0),
        %    d
        %    prev_dist
        %end
        
        
        % de embourbage
        if (not(isnan(prev_dist))) & (mod(frameID,20) == 0) & (d >= prev_dist),
            disp('I m stuck');
            forwBackVel = -forwBackVel;
            rotVel = -rotVel;
            leftRightVel = -leftRightVel;
            fsm = 'getout';
        else
            if (size(destination, 1) > 0) && (d < 3),
                fsm = 'checkpoint';
            else
                if d < 1,
                    fsm = 'checkpoint';
                else
                    alpha = abs(angdiff(angl, youbotEuler(3)));
                    if alpha < (pi / 8)
                        a = (pi - alpha)/pi*10;
                    elseif alpha < (pi / 16)
                        a = (pi - alpha)/pi*5;
                    else
                        a = (pi - alpha)/pi*1;
                    end
                    forwBackVel = -a*7*d*speed_coef;
                    rotVel = 20 * angdiff(angl, youbotEuler(3));
                    
                    if alpha < (pi / 10)
                        leftRightVel = 0 * angdiff(angl, youbotEuler(3));
                    end
                end
            end
        end
        if (isnan(prev_dist)) |  (mod(frameID,20) == 0),
            prev_dist = d;
        end
        
    elseif strcmp(fsm, 'getout'),
        if (mod(frameID,10) == 0),
            fsm = 'movingtopoint';
            prev_dist = NaN;
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

