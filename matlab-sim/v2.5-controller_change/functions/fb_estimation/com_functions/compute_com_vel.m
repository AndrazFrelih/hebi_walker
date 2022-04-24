function [comp] = compute_com_vel(Jlist,mlist,Qp)
    [Jc] = compute_centroidal_jac(Jlist,mlist);
    comp = Jc*Qp;
end

