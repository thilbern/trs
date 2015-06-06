    function map = map_update(map, youbotEuler, youbotPos, pts, contacts, xin, yin)
  
        % Learn the map, initialyze the transformation matrix 
        teta = youbotEuler(3);
        dx = youbotPos(1);
        dy = youbotPos(2);
        transf = [cos(teta) -sin(teta) dx ; sin(teta) cos(teta) dy ; 0 0 1];
        
        % Learn the map, fill the map matrix
        xwall = pts(1,contacts);
        ywall = pts(2,contacts);
        
        tmp = ones(3, size(xin, 2));
        tmp(1,:) = xin;
        tmp(2,:) = yin;
        tmp = transf * tmp;
        [tmp(1,:) tmp(2,:)] = wrapper_vrep_to_matrix(tmp(1,:), tmp(2,:));
        for m = tmp(1:2,:)
            map(int32(m(1)), int32(m(2))) =  map(int32(m(1)), int32(m(2))) - 1;
        end
    
        tmp = ones(3, size(xwall, 2));
        tmp(1,:) = xwall;
        tmp(2,:) = ywall;
        tmp = transf * tmp;
        [tmp(1,:) tmp(2,:)] = wrapper_vrep_to_matrix(tmp(1,:), tmp(2,:));
        for m = tmp(1:2,:)
            map(int32(m(1)), int32(m(2))) =  map(int32(m(1)), int32(m(2))) + 1;
        end
        
    end