function img = floodfill(x,y,img,value)
    % http://stackoverflow.com/questions/14238083/flood-fill-using-matlab
    import java.util.LinkedList
    q = LinkedList();

    initial = img(y,x);
    dims = size(img);
    if (value == initial) 
        error('cant flood fill as initial==value');
    end

    q.add([y, x]);

    while q.size() > 0

        pt = q.removeLast();
        y = pt(1);
        x = pt(2);
        
        img(y,x) = value;

        if (y < dims(1) && img(y+1, x) == initial) % step down
            q.add([y+1, x]); 
        end
        if (y > 1 && img(y-1, x) == initial) % step up
            q.add([y-1, x]); 
        end
        if (x < dims(2) && img(y, x+1) == initial) % step right
            q.add([y, x+1]); 
        end
        if (x > 1 && img(y, x-1) == initial) % step left
            q.add([y, x-1]); 
        end
    end
end