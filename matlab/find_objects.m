function xyztable = find_objects (vrep, id, h, youbotPos, youbotEuler, tables, xyztable)
    % Initialyze the transformation matrix RGB rotation
    transf0 = [1 0 0 ; 0 0 -1 ; 0 1 0];
    
    % Initialyze the transformation matrix youBot to Vrep
    teta = youbotEuler(3);
    dx = youbotPos(1);
    dy = youbotPos(2);
    transf2 = [cos(teta) -sin(teta) dx ; sin(teta) cos(teta) dy ; 0 0 1];
    
    angl = compute_photo_angle (youbotPos, youbotEuler, tables(2,:));
   
    % shift = [-5*pi/32 -4*pi/32 -3*pi/32 -pi/32 0  pi/32 3*pi/32 4*pi/32 5*pi/32];
    shift = [-3*pi/16 -2*pi/16 -pi/16 0 pi/16 2*pi/16 3*pi/16];
    for sh = shift,
        % Take picture
        [pts image] = take_picture(vrep, id, h, (angl-(pi/2) + sh), pi/16);
        [res camOri] = vrep.simxGetObjectOrientation(h.id, h.rgbdCasing, h.ref,  vrep.simx_opmode_oneshot_wait); 
        vrchk(vrep, res);
        
        % Initialyze the transformation matrix RGB to YOUBOT
        teta = camOri(3)+(pi/2);
        dx = 0;
        dy = -0.25;
        transf1 = [cos(teta) -sin(teta) dx ; sin(teta) cos(teta) dy ; 0 0 1];

        % Apply transformations
        pts = transf0 * pts;
        tmp =  pts(3,:);
        pts(3,:) = 1;
        pts = transf1 * pts;
        pts(3,:) = 1;
        pts = transf2 * pts;
        pts(3,:) = tmp + 0.2257;
        
        % Add new points to existing
        pts_small = pts(:,pts(3, :) < 0.180);
        pts_hight = pts(:,pts(3, :) > 0.185);
        xyztable = horzcat(xyztable, horzcat(pts_hight, pts_small));
    end
    
    subplot(324)
    cla
    plot3(xyztable(1,:), xyztable(2,:), xyztable(3,:), '*');
    axis equal;
end