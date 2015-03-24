function path_out = reduce_path (path_in)
    err = .2;
    
    if size(path_in, 1) < 3,
        path_out = path_in;
    else
        derv = zeros(size(path_in, 1)-1, 1);
        keep = 1:(size(path_in, 1)-1);
        
        for i=1:size(path_in, 1)-1,
            derv(i) = (path_in(i+1, 1)-path_in(i, 1) / (path_in(i+1, 2)-path_in(i, 2)));
        end
        
        for i=2:size(path_in, 1)-1,
            if derv(i-1) == derv(i),
                keep(i) = 0;
            end
        end
        keep = [keep(keep > 0) size(path_in, 1)];
        
        path_out = path_in(keep,:);
    
    end
end