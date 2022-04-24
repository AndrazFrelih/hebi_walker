function [cin,ceq] = nlcstr_mot_pl(z, phase, wshift, FKtask)
    %no nonlinear inequality constraints
    cin = -1;
    
    %optimization variables
    x = z(1:3);
    qL = z(4:9);
    qR = z(10:15);
    xp = z(16:18);
    qLp = z(19:24);
    qRp = z(25:30);
    
    %base to world
    xfull = [x;zeros(3,1)];
    Hwb = Hbase_in(xfull);
    
    %transform parametrised velocities (including euler angle velocities) to generalised ones
    gen_vel = FB_vel_map(xfull(4:6),[xp;zeros(3,1);qLp;qRp]);
    
    %divide the task
    FKtaskL = FKtask(1:end/2);
    FKtaskR = FKtask(end/2+1:end);
    
    if phase=='d'
        %double support phase (tasks for both feet)
        HabsL = H_abs_bL(qL);
        HabsR = H_abs_bR(qR);
        
        %compute Fkine and Jacobians
        [Fk_L,Jwee_L] = get_EE_task(Hwb,HabsL,'l');
        [Fk_R,Jwee_R] = get_EE_task(Hwb,HabsR,'r');
        
        %compute FDkine and write down constraint equations
        ceq = [
            FKtaskL-Fk_L;
            FKtaskR-Fk_R;
            Jwee_L*gen_vel;
            Jwee_R*gen_vel;
        ];
    else
        %single support phase
        if wshift=='l'
            %weight shifted to the LEFT
            HabsL = H_abs_bL(qL);
            %compute Fkine and Jacobians
            [Fk_L,Jwee_L] = get_EE_task(Hwb,HabsL,'l');
            %compute FDkine and write down constraint equations
            ceq = [
                FKtaskL-Fk_L;
                Jwee_L*gen_vel;
            ];
        else
            %weight shifted to the RIGHT
            HabsR = H_abs_bR(qR);
            %compute Fkine and Jacobians
            [Fk_R,Jwee_R] = get_EE_task(Hwb,HabsR,'r');
            %compute FDkine and write down constraint equations
            ceq = [
                FKtaskR-Fk_R;
                Jwee_R*gen_vel;
            ];
        end
    end
end

