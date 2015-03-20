function mout = map_refactor (map)
    mout = zeros(size(map)) + 0.5;
    err = 0;
    
    indices_pos = find(map > 0 + err);
    mout(indices_pos) = map(indices_pos);
    
    indices_neg = find(map < 0 - err);
    mout(indices_neg) = map(indices_neg);
    
    % inflate walls
    for ind = indices_pos',
        [i j] = ind2sub(size(map), ind);
        
        if (i > 2),
            mout(i-1, j) = err + 1;
        end
        
        if (j > 2),
            mout(i, j-1) = err + 1;
        end
        
        if (i < size(mout, 1)),
            mout(i+1, j) = err + 1;
        end
        
        if (j < size(mout, 2)),
            mout(i, j+1) = err + 1;
        end
    end
    
    
    % find start point to process gaps
    istart = 0;
    jstart = 0;
    for ind = indices_neg',
        [i j] = ind2sub(size(map), ind);
        
        if (i > 2),
            if (mout(i-1, j) == 0.5),
                istart = i-1;
                jstart = j;
                break;
            end
        end
        
        if (j > 2),
            if (mout(i, j-1) == 0.5),
                istart = i;
                jstart = j-1;
                break;
            end
        end
        
        if (i < size(mout, 1)),
            if (mout(i+1, j) == 0.5),
                istart = i+1;
                jstart = j;
                break;
            end
        end
        
        if (j < size(mout, 2)),
            if ( mout(i, j+1) == 0.5),
                istart = i;
                jstart = j+1;
                break;
            end
        end
    end
    
    mout = floodfill(istart, jstart, mout, 0);
    mout(mout == 0.5) = 1000;
        
        
    
    