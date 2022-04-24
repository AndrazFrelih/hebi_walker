function [cin,ceq] = nlcstr_init(x, qL, qR, TaskL, TaskR)
    %no inequality constraints
    cin = -1;
    
    %equality constraints
    [HabsL] = H_abs_bL(qL);
    [HabsR] = H_abs_bR(qR);
    HbeeL = reshape(HabsL(end,:,:),4,4);
    HbeeR = reshape(HabsR(end,:,:),4,4);
    
    Hfootfix = Hcpts; %generated function
    
    xfull = [
        x;          %only position of the fb is important
        zeros(3,1); %base has to be upright
    ];
    
    Hwb = Hbase_in(xfull);
    
    %end effector transformation
    HLfc = Hwb*HbeeL;
    HRfc = Hwb*HbeeR;

    Fkine = [
        opt_Fkine(HLfc,Hfootfix);
        opt_Fkine(HRfc,Hfootfix);
    ];

    %end effector task    
    Task = [
        TaskL;
        TaskR;
    ];
    
    ceq = Task-Fkine; %hard equality nonlinear constraint of a type X=Fkine(q)
end

