function [zmp_glo,zmpL,zmpR] = compute_zmp_full(FSL, FSR, foot_data, Hw_eeL, Hw_eeR, thr, simpleZMP)
    %this is the computation carried out by the control algorithm
    FmL = FSL.Fm;
    FmR = FSR.Fm;
    FmL_tot = sum(FmL);
    FmR_tot = sum(FmR);
    
    Pm = foot_data.fsen.Pmeas;
    Pe = foot_data.fsen.Pedge;
    AinvX = foot_data.zmp.Am.xinv;
    AinvY = foot_data.zmp.Am.yinv;
    Apseud = foot_data.zmp.Am.finv;

    if simpleZMP
        [zmp_L_glo,zmp_L_loc] = compute_global_zmp_simple(FmL,Pm,Hw_eeL); 
        [zmp_R_glo,zmp_R_loc] = compute_global_zmp_simple(FmR,Pm,Hw_eeR);
    else
        %remap forces from measurement position to the edge of the foot (to
        %use the entire foot size)
        %origin points = Pedge
        %target points = Pmeas
        [~,zmp_L_glo,zmp_L_loc,~] = force_remapping(FmL, AinvX, AinvY, Apseud, Pm, Pe, Hw_eeL(1:3,1:3), Hw_eeL(1:3,4), thr, 't');
        [~,zmp_R_glo,zmp_R_loc,~] = force_remapping(FmR, AinvX, AinvY, Apseud, Pm, Pe, Hw_eeR(1:3,1:3), Hw_eeR(1:3,4), thr, 't');
    end

    zmp_glo = (zmp_L_glo*FmL_tot + zmp_R_glo*FmR_tot) / (FmL_tot + FmR_tot);
    zmpL.glo = zmp_L_glo;
    zmpL.loc = zmp_L_loc;
    zmpR.glo = zmp_R_glo;
    zmpR.loc = zmp_R_loc;
end

