function[i, j] = wrapper_vrep_to_matrix (x, y)
    % World size
    offset = 2;
    halfwsize = 7.45;
    wsize = 7.45 * 2;
    
    j = ((x + halfwsize) .* 4) + offset;
    i = ((wsize - (y + halfwsize)) .* 4) + offset;
end