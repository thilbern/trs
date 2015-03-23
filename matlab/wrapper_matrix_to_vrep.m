function[x, y] = wrapper_matrix_to_vrep (i, j)
    % World size
    offset = 2;
    halfwsize = 7.45;
    wsize = 7.45 * 2;
    
    x = ((j - offset) ./ 4) - halfwsize;
    y = ((-i + offset) ./ 4) + wsize - halfwsize;
end