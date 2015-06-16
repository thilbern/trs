function coord = wrapper_vrep_to_arm(youbotPos, youbotEuler, coord_vrep)
    % Compute transformations matrix
    teta = -youbotEuler(3);
    dx = youbotPos(1);
    dy = youbotPos(2);
    transf0 = [cos(teta) -sin(teta) dx ; sin(teta) cos(teta) dy ; 0 0 1];
    teta = 0;
    dx = 0;
    dy = -0.1662;
    transf1 = [cos(teta) -sin(teta) dx ; sin(teta) cos(teta) dy ; 0 0 1];
    
    teta = youbotEuler(3);
    dx = youbotPos(1);
    dy = youbotPos(2);
    transf3 = [cos(teta) -sin(teta) dx ; sin(teta) cos(teta) dy ; 0 0 1];
    
    tmp = [0 ; 0 ; 1]
    tmp1 = transf0 * tmp
    tmp(3) = 1;
    tmp2 = transf3 * tmp
    
    % Apply transformations
    tmp = coord_vrep(3);
    coord_vrep(3) = 1;
    coord = transf0 * coord_vrep';
    coord(3) = 1;
    coord = transf1 * coord;
    coord(3) = tmp - 0.1446;
end