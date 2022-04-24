function [traj] = generate_lin_traj(tpts,tvec,Xst,Xen)
    Tst = tpts(1);
    Ten = tpts(2);
    dT = Ten-Tst;
    K = (Xen-Xst)/dT;
    n = (Xst*Ten - Xen*Tst)/dT;
    traj = K.*tvec + n;
end

