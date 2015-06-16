function angl = compute_photo_angle (youbotPos, youbotEuler, to) 
    % Initialyze the transformation matrix youBot to Vrep
    teta = youbotEuler(3);
    dx = youbotPos(1);
    dy = youbotPos(2);
    transf = [cos(teta) -sin(teta) dx ; sin(teta) cos(teta) dy ; 0 0 1];
        
    % Compute the direction to take photos
    camVrep = transf * [0 ; -0.25 ; 1];   
    [i j] = wrapper_vrep_to_matrix(camVrep(1), camVrep(2));
    from = double([i j]);
    angl = compute_angle(from, to);
    angl = angdiff(angl, youbotEuler(3));
return