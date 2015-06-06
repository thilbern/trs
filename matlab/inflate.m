function map = inflate (map, walls, val, goal)
    tresh = 1;
    
    % inflate walls
    for ind = walls',
        [i j] = ind2sub(size(map), ind);
        
        if (i > 2 && (pdist([goal ; [(i-1) j]], 'euclidean') > tresh)),
            map(i-1, j) = val;
        end
        
        if (j > 2 && (pdist([goal ; [i (j-1)]], 'euclidean') > tresh)),
            map(i, j-1) = val;
        end
        
        if (i < size(map, 1) && (pdist([goal ; [(i+1) j]], 'euclidean') > tresh)),
            map(i+1, j) = val;
        end
        
        if (j < size(map, 2) && (pdist([goal ; [i (j+1)]], 'euclidean') > tresh)),
            map(i, j+1) = val;
        end
        
        %diag
        if (i > 2 && j >2 && (pdist([goal ; [(i-1) (j-1)]], 'euclidean') > tresh)),
            map(i-1, j-1) = val;
        end
        
        if (i > 2 && j < size(map, 2) && (pdist([goal ; [(i-1) (j+1)]], 'euclidean') > tresh)),
            map(i-1, j+1) = val;
        end
        
        if (i < size(map, 1) && j > 2 && (pdist([goal ; [(i+1) (j-1)]], 'euclidean') > tresh)),
            map(i+1, j-1) = val;
        end
        
        if (i < size(map, 1) && j < size(map, 2) && (pdist([goal ; [(i+1) (j+1)]], 'euclidean') > tresh)),
            map(i+1, j+1) = val;
        end
    end
end