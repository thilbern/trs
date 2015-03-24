function print_map(plotid, map, to, robot_path)
    robot_path = [to;robot_path];
    subplot(plotid)
    [row_neg, col_neg] = find(map < 0);
    [row_zero, col_zero] = find(map == 0);
    [row_pos, col_pos] = find(map > 0);
    [x_neg, y_neg] = wrapper_matrix_to_vrep(row_neg, col_neg);
    [x_zero, y_zero] = wrapper_matrix_to_vrep(row_zero, col_zero);
    [x_pos, y_pos] = wrapper_matrix_to_vrep(row_pos, col_pos);
    
    if size(robot_path, 1) > 0,
        [pathx pathy] = wrapper_matrix_to_vrep(robot_path(:,1), robot_path(:,2));
        plot(x_neg, y_neg, '.c', x_zero, y_zero, '.r', x_pos, y_pos, '.b', pathx, pathy, 'ok')
    else
        plot(x_neg, y_neg, '.c', x_zero, y_zero, '.r', x_pos, y_pos, '.b')
    end
    
    axis([-8 8 -8 8]);
    axis equal;
    drawnow;
end