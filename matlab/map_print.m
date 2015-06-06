function map_print(plotid, map, points)
    subplot(plotid)
    [row_neg, col_neg] = find(map < 0);
    [row_zero, col_zero] = find(map == 0);
    [row_pos, col_pos] = find(map > 0);
    [x_neg, y_neg] = wrapper_matrix_to_vrep(row_neg, col_neg);
    [x_zero, y_zero] = wrapper_matrix_to_vrep(row_zero, col_zero);
    [x_pos, y_pos] = wrapper_matrix_to_vrep(row_pos, col_pos);
    
    if size(points, 1) > 0,
        [pointx pointy] = wrapper_matrix_to_vrep(points(:,1), points(:,2));
        plot(x_neg, y_neg, '.c', x_zero, y_zero, '.r', x_pos, y_pos, '.b', pointx, pointy, 'ok')
    else
        plot(x_neg, y_neg, '.c', x_zero, y_zero, '.r', x_pos, y_pos, '.b')
    end
    
    axis([-8 8 -8 8]);
    axis equal;
    drawnow;
end