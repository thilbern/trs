function path_out = reduce_path (path_in, map)

    if (size(path_in, 1) < 3)
        path_out = path_in;
    else
        i1 = path_in(1,1);
        j1 = path_in(1,2);
        i2 = path_in(end,1);
        j2 = path_in(end,2);
        
        res= bresenham(i1,j1,i2,j2);
        i = res(:,1);
        j = res(:,2);
        ind = sub2ind(size(map), i, j);
        
        conflict = find(map(ind) == 1);
        if (size(conflict, 1) > 0) % We have a conflict
            out1 = reduce_path(path_in(1:int32(end/2),:), map);
            out2 = reduce_path(path_in(int32(end/2)+1:end,:), map);
            path_out = [out1 ; out2 ];
        else
            path_out = [[i1 j1] ; [i2 j2]];
        end
    end
end