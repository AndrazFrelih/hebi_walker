function [Fkine] = get_EE_task_static(Hwb,Hlist)
    %end effector transformation to base
    Hb_ee = reshape(Hlist(7,:,:),4,4);

    %end effector transformation to world
    Hw_ee = Hwb*Hb_ee;

    %generate outputs
    Hffix = Hcpts;
    Fkine = opt_Fkine(Hw_ee,Hffix);
end

