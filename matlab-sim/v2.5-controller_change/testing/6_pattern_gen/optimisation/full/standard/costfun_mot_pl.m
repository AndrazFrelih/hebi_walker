function [J] = costfun_mot_pl(z, phase, wshift, C_des, Cp_des, FKtask, FDKtask, qp_last, mlist)
    %optimization variables
    x = z(1:3);
    qL = z(4:9);
    qR = z(10:15);
    xp = z(16:18);
    qLp = z(19:24);
    qRp = z(25:30);
    
    %weights
    wi = {
        100000,...  %com position
        1000,...  %com velocity
        0.1,...   %joint angle velocity difference
        1,...     %swing foot position
        [1, 0.01]  %swing foot tra. and rot. velocities
    };
    
    xfull = [x;zeros(3,1)];
    
    %compute COM position and velocity
    [C,Cp] = compute_com_kin(xfull,qL,qR,xp,qLp,qRp,mlist,'w');
    
    %stage independent costs
    J1a = wi{1}*sq_mat2(C_des - C); %try to follow the COM position closely
    J1b = wi{2}*sq_mat2(Cp_des - Cp); %try to follow the COM velocity closely
    J1c = wi{3}*sq_mat2([qLp;qRp] - qp_last); %keep accelerations bounded
    
    %sum of stage independent costs 
    J1 = J1a+J1b+J1c;
    
    %stage dependent cost
    if phase=='d'
        %double support phase (tasks for both feet)
        J2 = 0;
    else
        Hwb = Hbase_in(xfull);
        %transform parametrised velocities (including euler angle velocities) to generalised ones
        gen_vel = FB_vel_map(xfull(4:6),[xp;zeros(3,1);qLp;qRp]);
        %single support phase
        if wshift=='l'
            %swing foot is the RIGHT one
            HabsR = H_abs_bR(qR);
            [Fk_R,Jwee_R] = get_EE_task(Hwb,HabsR,'r');
            J2a = wi{4}*sq_mat2(FKtask-Fk_R); %follow RIGHT swing foot position trajectory
            J2b1 = wi{5}(1)*sq_mat2(FDKtask(1:3)-Jwee_R(1:3,:)*gen_vel); %follow RIGHT swing foot velocity profile (tra vel)
            J2b2 = wi{5}(2)*sq_mat2(FDKtask(4:6)-Jwee_R(4:6,:)*gen_vel); %follow RIGHT swing foot velocity profile (rot vel)
        elseif wshift=='r'
            %swing foot is the LEFT one
            HabsL = H_abs_bL(qL);
            [Fk_L,Jwee_L] = get_EE_task(Hwb,HabsL,'l');
            J2a = wi{4}*sq_mat2(FKtask-Fk_L); %follow LEFT swing foot position trajectory
            J2b1 = wi{5}(1)*sq_mat2(FDKtask(1:3)-Jwee_L(1:3,:)*gen_vel); %follow LEFT swing foot velocity profile (tra vel)
            J2b2 = wi{5}(2)*sq_mat2(FDKtask(4:6)-Jwee_L(4:6,:)*gen_vel); %follow LEFT swing foot velocity profile (rot vel)
        end
        %sum of stage dependent costs 
        J2 = J2a + J2b1 + J2b2;
    end
    
    J = J1 + J2;
end

