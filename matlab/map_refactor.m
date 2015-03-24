function mout = map_refactor (map, goal)
    mout = zeros(size(map)) + 0.5;
    err = 0;
    tresh = 1;
    %mur
    indices_pos = find(map > 0 + err);
    mout(indices_pos) = map(indices_pos);
    
    indices_neg = find(map < 0 - err);
    
    % inflate walls
    for ind = indices_pos',
        [i j] = ind2sub(size(map), ind);
        
        if (i > 2 && (pdist([goal ; [(i-1) j]], 'euclidean') > tresh)),
            mout(i-1, j) = err + 1;
        end
        
        if (j > 2 && (pdist([goal ; [i (j-1)]], 'euclidean') > tresh)),
            mout(i, j-1) = err + 1;
        end
        
        if (i < size(mout, 1) && (pdist([goal ; [(i+1) j]], 'euclidean') > tresh)),
            mout(i+1, j) = err + 1;
        end
        
        if (j < size(mout, 2) && (pdist([goal ; [i (j+1)]], 'euclidean') > tresh)),
            mout(i, j+1) = err + 1;
        end
        
        %diag
        
        if (i > 2 && j >2 && (pdist([goal ; [(i-1) (j-1)]], 'euclidean') > tresh)),
            mout(i-1, j-1) = err + 1;
        end
        
        if (i > 2 && j < size(mout, 2) && (pdist([goal ; [(i-1) (j+1)]], 'euclidean') > tresh)),
            mout(i-1, j+1) = err + 1;
        end
        
        if (i < size(mout, 1) && j > 2 && (pdist([goal ; [(i+1) (j-1)]], 'euclidean') > tresh)),
            mout(i+1, j-1) = err + 1;
        end
        
        if (i < size(mout, 1) && j < size(mout, 2) && (pdist([goal ; [(i+1) (j+1)]], 'euclidean') > tresh)),
            mout(i+1, j+1) = err + 1;
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
    mout = floodfill(istart, jstart, mout, -8);
   
    mout(mout == 0.5) = 1000;
end
        
        
    
    