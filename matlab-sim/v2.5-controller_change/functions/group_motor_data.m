function [LL,RL] = group_motor_data(MSL,MSR)
    %% group measurements
    % left leg
    jL1 = MSL.mot1;
    jL2 = MSL.mot2;
    jL3_1 = MSL.mot3_1;
    jL3_2 = MSL.mot3_2;
    jL4_1 = MSL.mot4_1;
    jL4_2 = MSL.mot4_2;
    jL5 = MSL.mot5;
    jL6 = MSL.mot6;

    %right leg
    jR1 = MSR.mot1;
    jR2 = MSR.mot2;
    jR3_1 = MSR.mot3_1;
    jR3_2 = MSR.mot3_2;
    jR4_1 = MSR.mot4_1;
    jR4_2 = MSR.mot4_2;
    jR5 = MSR.mot5;
    jR6 = MSR.mot6;

    % measured angles 
    qL = [jL1.q,jL2.q,jL3_1.q,jL4_2.q,jL5.q,jL6.q]';
    qR = [jR1.q,jR2.q,jR3_1.q,jR4_2.q,jR5.q,jR6.q]';
    % measured agular velocities
    qLp = [jL1.qp,jL2.qp,jL3_1.qp,jL4_2.qp,jL5.qp,jL6.qp]';
    qRp = [jR1.qp,jR2.qp,jR3_1.qp,jR4_2.qp,jR5.qp,jR6.qp]';
    
    % accelerometer measurements
    aL = [jL1.a,jL2.a,jL3_1.a,jL4_2.a,jL5.a,jL6.a];
    aR = [jR1.a,jR2.a,jR3_1.a,jR4_2.a,jR5.a,jR6.a];
    
    % gyro measurements
    wL = [jL1.w,jL2.w,jL3_1.w,jL4_2.w,jL5.w,jL6.w];
    wR = [jR1.w,jR2.w,jR3_1.w,jR4_2.w,jR5.w,jR6.w];
    
    LL = struct("q", qL, "qp", qLp, "a", aL, "w", wL);
    RL = struct("q", qR, "qp", qRp, "a", aR, "w", wR);
end

