function [cin,ceq] = nlcstr_mot_pl_static(z, phase, wshift, FKtask)
    %no nonlinear inequality constraints
    cin = -1;

    %optimization variables
    x = z(1:3);
    qL = z(4:9);
    qR = z(10:15);
    
    %base to world
    xfull = [x;zeros(3,1)];
    Hwb = Hbase_in(xfull);
    
    %divide the task
    FKtaskL = FKtask(1:end/2);
    FKtaskR = FKtask(end/2+1:end);
    
    if phase=='d'
        %double support phase (tasks for both feet)
        HabsL = H_abs_bL(qL);
        HabsR = H_abs_bR(qR);
        
        %compute Fkine and Jacobians
        Fk_L = get_EE_task_static(Hwb,HabsL);
        Fk_R = get_EE_task_static(Hwb,HabsR);
        
        %compute FDkine and write down constraint equations
        ceq = [
            FKtaskL-Fk_L;
            FKtaskR-Fk_R;
        ];
    else
        %single support phase
        if wshift=='l'
            %weight shifted to the LEFT
            HabsL = H_abs_bL(qL);
            %compute Fkine and Jacobians
            Fk_L = get_EE_task_static(Hwb,HabsL);
            %compute FDkine and write down constraint equations
            ceq = FKtaskL-Fk_L;
            
        else
            %weight shifted to the RIGHT
            HabsR = H_abs_bR(qR);
            %compute Fkine and Jacobians
            Fk_R = get_EE_task_static(Hwb,HabsR);
            %compute FDkine and write down constraint equations
            ceq = FKtaskR-Fk_R;
            
        end
    end
    
end

