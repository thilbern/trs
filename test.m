    elseif strcmp(fsm, 'findangle'),
        [i j] = wrapper_vrep_to_matrix(youbotPos(1), youbotPos(2));
        from = double([i j]);
        to = double(to);
        deltax = to(1) - from(1);
        deltay = to(2) - from(2);
        angl = atan2(deltay, deltax);
        fsm = 'rotate';
    
    elseif strcmp(fsm, 'rotate'),
        forwBackVel = 0;
        leftRightVel = 0;
        rotVel = 10*angdiff(angl, youbotEuler(3));
        if abs(angdiff(angl, youbotEuler(3))) < 1/180*pi,
            rotVel = 0;
            fsm = 'drive';
        end

    elseif strcmp(fsm, 'drive'),
        forwBackVel = 0;
        leftRightVel = 0;
        rotVel = 0;
        tresh = 2;
        
        [i j] = wrapper_vrep_to_matrix(youbotPos(1), youbotPos(2));
        d = pdist([to ; [i j]], 'euclidean');
        
        if (size(destination, 1) > 0) && (d < 2) && (map(destination(1), destination(2)) < -tresh || map(destination(1), destination(2)) > tresh),
            % robot_path = [];
            fsm = 'checkpoint';
        else
            if d < .5,
                fsm = 'checkpoint';
            else
                forwBackVel = -20*d;
                rotVel = 20*angdiff(angl, youbotEuler(3));
            end
            
            % if d > 1,
            %    if (abs(angdiff(angl, youbotEuler(3))) > 4/180*pi),
            %        fsm = 'findangle';
            %    end
            %    rotVel = 20*angdiff(angl, youbotEuler(3));
            %
            % elseif d > .5,
            %     fsm = 'findangle';
            % else
            %     forwBackVel = 0;
            %     rotVel = 0;
            %     fsm = 'checkpoint';
            % end
        end