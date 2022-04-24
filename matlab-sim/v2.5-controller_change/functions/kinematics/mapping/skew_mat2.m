function [out] = skew_mat2(mat)
    out = (mat - mat')/2;
end

