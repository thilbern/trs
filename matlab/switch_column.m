function out = switch_column(in, j1, j2)
        out = zeros(size(in));
        out(:,j1) = in(:,j2);
        out(:,j2) = in(:,j1);