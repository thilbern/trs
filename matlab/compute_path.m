function robot_path = compute_path(map, from, to)
    ori = switch_column(from, 1, 2);
    dest = switch_column(to, 1, 2);
    
    % Initialyze the navigation object
    dx = DXform(map, 'metric', 'cityblock', 'inflate', 1);
    dx.plan(dest);
    
    % Find the robot_path to the start point
    robot_path = dx.path(ori);
    robot_path = switch_column(robot_path, 1, 2);
    robot_path = reduce_path (robot_path, map);
end