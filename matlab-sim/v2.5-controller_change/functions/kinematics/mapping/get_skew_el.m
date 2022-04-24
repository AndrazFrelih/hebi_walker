function [vect] = get_skew_el(mat)
    v1 = -mat(2,3);
    v2 = +mat(1,3);
    v3 = -mat(1,2);
    vect = [v1;v2;v3];
end

