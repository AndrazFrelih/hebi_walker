function [zmp_glo,zmp_loc] = compute_global_zmp_simple(Fm,Pi,Hw_ee)
    zmp_loc = compute_local_zmp_simple(Fm,Pi);
    zmpHtf = Hw_ee * [zmp_loc;0;1];
    zmp_glo = zmpHtf(1:2);
end

