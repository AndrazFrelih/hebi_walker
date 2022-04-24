function [J] = costfun_mot_pl_static(z, phase, wshift, C_des, FKtask, mlist)
    %optimization variables
    x = z(1:3);
    qL = z(4:9);
    qR = z(10:15);
    
    %weights
    wi = {
        1000,...  %com position
        10     %swing foot position
    };
    
    xfull = [x;zeros(3,1)];
    
    %compute COM position and velocity
    [C] = compute_com_pos2(xfull,qL,qR,mlist,'w');
    
    %stage independent costs
    J1 = wi{1}*sq_mat2(C_des - C); %try to follow the COM position closely
    
    %stage dependent cost
    if phase=='d'
        %double support phase (tasks for both feet)
        J2 = 0;
    else
        Hwb = Hbase_in(xfull);
        %single support phase
        if wshift=='l'
            %swing foot is the RIGHT one
            HabsR = H_abs_bR(qR);
            Fk_R = get_EE_task_static(Hwb,HabsR);
            J2 = wi{2}*sq_mat2(FKtask-Fk_R); %follow RIGHT swing foot position trajectory
        elseif wshift=='r'
            %swing foot is the LEFT one
            HabsL = H_abs_bL(qL);
            Fk_L = get_EE_task_static(Hwb,HabsL);
            J2 = wi{2}*sq_mat2(FKtask-Fk_L); %follow LEFT swing foot position trajectory
        end
    end
    
    J = J1 + J2;
end

