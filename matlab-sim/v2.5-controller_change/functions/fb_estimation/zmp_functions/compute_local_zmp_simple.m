function [zmp] = compute_local_zmp_simple(Fm,Pi)
    zmp = Pi'*Fm / sum(Fm);
end

