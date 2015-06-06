function [tables baskets baskets_entry] = find_basket (map)
    baskets = [];
    baskets_entry = [];
    tables = []
    mask1 = [1 1 1 1 1 1; 
             1 1 1 1 0 0; 
             1 1 1 1 0 0; 
             1 1 1 1 0 0; 
             1 0 0 0 0 0; 
             1 0 0 0 0 0];
    mask2 = [1 1 1 1 1 1; 
             0 0 1 1 1 1; 
             0 0 1 1 1 1; 
             0 0 1 1 1 1; 
             0 0 0 0 0 1; 
             0 0 0 0 0 1];
    mask3 = [0 0 0 0 0 1;
             0 0 0 0 0 1;
             0 0 1 1 1 1;
             0 0 1 1 1 1;
             0 0 1 1 1 1;
             1 1 1 1 1 1];
    mask4 = [1 0 0 0 0 0;
             1 0 0 0 0 0;
             1 1 1 1 0 0; 
             1 1 1 1 0 0; 
             1 1 1 1 0 0;
             1 1 1 1 1 1];
    table = [0 0 0 0 0 0;
             0 1 1 1 1 0;
             0 1 1 1 1 0; 
             0 1 1 1 1 0; 
             0 1 1 1 1 0;
             0 0 0 0 0 0];
    map_len = size(map)
    for i = 1:map_len(1)-5,
        for j = 1:map_len(2)-5,
            tmp = map(i:(i+5),j:(j+5));
            if sum(sum(xor(tmp, mask1))) < 3
                baskets = [(i+2) (j+2); baskets];
                baskets_entry = [(i+5) (j+5); baskets_entry];
                
            elseif sum(sum(xor(tmp, mask2))) < 3
                baskets = [(i+3) (j+2) ; baskets];
                baskets_entry = [(i+5) j ; baskets_entry];
                
            elseif sum(sum(xor(tmp, mask3))) < 3
                baskets = [(i+4) (j+4); baskets];
                baskets_entry = [i j; baskets_entry];
                
            elseif sum(sum(xor(tmp, mask4))) < 3
                baskets = [(i+3) (j+2); baskets];
                baskets_entry = [i (j+5); baskets_entry];
                
            elseif sum(sum(xor(tmp, table))) < 3
                tables = [(i+2) (j+2); tables];
            end
        end
    end
end