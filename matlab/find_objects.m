function find_objects (vrep, id, h, youbotPos, youbotEuler, tables)

    tables
    
    [i j] = wrapper_vrep_to_matrix(youbotPos(1), youbotPos(2));
    from = double([i j]);
    deltax = tables(2,1) - from(1);
    deltay = tables(2,2) - from(2);
    angl = atan2(deltay, deltax)
    angl = angdiff(angl, youbotEuler(3))
   
    [pts image] = take_picture(vrep, id, h, (angl-(pi/2)-(pi/8)), pi/4, -0.04);
    
    % Plot 3D points and save them to file
    subplot(324)
    plot3(pts(1,:), pts(2,:), pts(3,:), '*');
    axis equal;
    view([-169 -46]);
    fileID = fopen('pc.xyz','w');
    fprintf(fileID,'%f %f %f\n',pts);
    fclose(fileID);
    fprintf('Read %i 3D points, saved to pc.xyz.\n', max(size(pts)));
    
    % Display the RGB image
    subplot(325)
    imshow(image);
    drawnow;
    
    pause(5)
    [pts image] = take_picture(vrep, id, h, (angl-(pi/2)+(pi/8)), pi/4, -0.04);
    
    % Plot 3D points and save them to file
    subplot(324)
    cla
    plot3(pts(1,:), pts(2,:), pts(3,:), '*');
    axis equal;
    view([-169 -46]);
    fileID = fopen('pc.xyz','w');
    fprintf(fileID,'%f %f %f\n',pts);
    fclose(fileID);
    fprintf('Read %i 3D points, saved to pc.xyz.\n', max(size(pts)));
    
    % Display the RGB image
    subplot(325)
    imshow(image);
    drawnow;
end