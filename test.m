            [row_neg, col_neg] = find(map < 0);
            [row_zero, col_zero] = find(map == 0);
            [row_pos, col_pos] = find(map > 0);
            indices = sub2ind(size(map2), col_neg, row_neg);
            map2(indices) = 0;
            indices = sub2ind(size(map2), col_zero, row_zero);
            map2(indices) = 0;
            indices = sub2ind(size(map2), col_pos, row_pos);
            map2(indices) = 1;
            
            % Get the "goal" point
            [i j] = wrapper_vrep_to_matrix(youbotPos(1), youbotPos(2));
            goal = [i j]
            
            %Initialyze the navigation object
            dx = DXform(map2, 'metric', 'cityblock');
            dx.plot()
            pause(5)
            dx.plan(goal);
            dx.plot()
            pause(5)
            
            % Get the "start" point
            i = find(map==0);
            tmp = dx.distancemap(i);
            i2 = find(tmp == min(tmp));

            if size(i2, 1) > 0,
                [x y] = ind2sub(size(map), i(i2(1)));
                start = [y x]
                
                % Find the path to the start point
                dx.path(start);
                path = dx.path(start);
                path = [flipud(path) ; start] ./ 4
                pause(5);
            else
                disp('No start point');
                fsm = 'finished';
            end
            
            
            
            
            
            

            izeros = find(map == 0);
            
            map2(find(map < 0)) = 0;
            map2(izeros) = 0;
            map2(find(map > 0)) = 1;
            
            % Get the "goal" point
            [i j] = wrapper_vrep_to_matrix(youbotPos(1), youbotPos(2));
            goal = [i j]
            
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
                [x y] = ind2sub(size(map), izeros(i2(1)));
                start = [y x]
                
                % Find the path to the start point
                dx.path(start);
                path = dx.path(start);
                path = [flipud(path) ; start] ./ 4
                pause(5);
            else
                disp('No start point');
                fsm = 'finished';
            end