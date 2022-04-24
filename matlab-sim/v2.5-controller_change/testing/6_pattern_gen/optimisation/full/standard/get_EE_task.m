function [Fkine,Jac] = get_EE_task(Hwb,Hlist,side)
    %end effector transformation to base
    Hb_ee = reshape(Hlist(7,:,:),4,4);

    %end effector transformation to world
    Hw_ee = Hwb*Hb_ee;

    %create jacobians that map to the base frame
    Jb_ee = compute_jacobian(Hlist,7);

    %transform them to the world frame
    Jw_ee = compute_jac_world_single(Hwb,Jb_ee,Hb_ee,side);
    
    %generate outputs
    Hffix = Hcpts;
    Fkine = opt_Fkine(Hw_ee,Hffix);
    Jac = Jw_ee;
end

