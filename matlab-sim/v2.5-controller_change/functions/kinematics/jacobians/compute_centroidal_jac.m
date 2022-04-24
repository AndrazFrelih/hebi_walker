function [Jc] = compute_centroidal_jac(Jlist,mlist)
    dim = size(Jlist,[2,3]);
    Jc = zeros(dim);
    M = sum(mlist);
    for i=1:size(mlist,1)
        Jc = Jc + reshape(Jlist(i,:,:),dim)*mlist(i);
    end

    Jc = Jc/M;
end

