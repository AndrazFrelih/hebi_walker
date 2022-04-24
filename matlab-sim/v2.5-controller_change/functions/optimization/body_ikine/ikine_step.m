function [xsol,wsol] = ikine_step(meas,ctrl,winit,config)
    dt = 1/config.fctrl;

    %% measured values
    % LIPM
    ZMP_m = meas.z;
    COM_m = meas.c;
    COMp_m = meas.cp;
    DCM_m = meas.dcm;
    DCMp_m = meas.dcmp;
    % EE - obtained from FKine
    pL_m = meas.tL;
    RL_m = meas.RL;
    pR_m = meas.tR;
    RR_m = meas.RR;
    % robot states
    Xfb_m = meas.xfb;
    Rfb_m = meas.efb;
    qL_m = meas.qL;
    qR_m = meas.qR;
    q_m = [qL_m;qR_m];
    
    %% commanded values
    % LIPM
    ZMP_c = ctrl.z;
    COM_c = ctrl.c;
    COMp_c = ctrl.cp;
    DCM_c = ctrl.dcm;
    DCMp_c = ctrl.dcmp;
    % EE - obtained from FKine
    pL_c = ctrl.tL;
    RL_c = ctrl.RL;
    pLp_c = ctrl.vL;
    pR_c = ctrl.tR;
    RR_c = ctrl.RR;
    pRp_c = ctrl.vR;
    % robot states
    Xfb_c = ctrl.xfb;
    Rfb_c = ctrl.efb;
    qL_c = ctrl.qL;
    qR_c = ctrl.qR;
    q_c = [qL_c;qR_c];
    
    %% compute optimisation targets
    % postural task - desired position of angles
    Ks_par = [1,1,1,1,1,1];
    Ks = diag([Ks_par,Ks_par]);
    qerr = q_m - q_c;
    qp_d = -Ks*qerr;
    
    % postural task - orientation of the floating base
    Kwfb_par = [1,1,1];
    Kwfb = diag(Kwfb_par);
    wfb_d = -Kwfb * get_skew_el(skew_mat2(Rfb_m*Rfb_c'));
    
    % leg task - position and orientation of the foot
    Kvleg_par = [1,1,1];
    Kwleg_par = [1,1,1];
    Kvleg = diag(Kvleg);
    Kwleg = diag(Kwleg_par);
    % left foot
    vL_ff = [
        pLp_c;
        zeros(3,1)
    ];
    vL_fb = [
        -Kvleg * (pL_m - pL_c);  %include the integrator
        -Kwleg * get_skew_el(skew_mat2(RL_m*RL_c'));
    ];    
    vL_d = vL_ff + vL_fb;
    
    % right foot
    vR_ff = [
        pRp_c;
        zeros(3,1)
    ];
    vR_fb = [
        -Kvleg * (pR_m - pR_c);  %include the integrator
        -Kwleg * get_skew_el(skew_mat2(RR_m*RR_c'));
    ];
    vR_d = vR_ff + vR_fb;
    
    % COM task
    Kcom_par = [1,1,1];
    Kcom = diag(Kcom_par);
    vcom_d = COMp_c - Kcom * (COM_m - COM_c); %include the integrator
    
    %% compute jacobians
    xst = [
        TG.state_vector{1}.fb.x(:,1);
        zeros(3,1);
        TG.state_vector{1}.joint.q.L(:,1);
        TG.state_vector{1}.joint.q.R(:,1);
    ];
    
    [Jw_com, Jw_eeL, Jw_eeR, Jw_b] = get_jacobians(xst,config.mlist);
    
    %% setup of Px, qx and Ax matrices and bounds
    % Px
    R1x = Jw_b(4:6,:);
    W1x = eye(min(size(R1x)))*1000;
    s1x = wfb_d;
    P1x = R1x'*W1x*R1x;
    q1x = R1x'*W1x*s1x;

    R2x = [
        zeros(Nq,6), eye(Nq);
    ];
    W2x = eye(min(size(R2x))); %do not include the task as here measurements are not available
    s2x = qp_d;
    P2x = R2x'*W2x*R2x;
    q2x = R2x'*W2x*s2x;

    Px = P1x + P2x;
    qx = q1x + q2x;

    % cost function
    Jcost = @(w)w'*Px*w+qx'*w;

    % Aeq and beq formulation
    A1eq = Jw_com(1:3,:);
    b1eq = vcom_d;

    A2eq = Jw_eeL;
    b2eq = vL_d;

    A3eq = Jw_eeR;
    b3eq = vR_d;
        
    % bounds
    qpmax = 10;
    lb = [-Inf(6,1);-qpmax*ones(Nq,1)];
    ub = [+Inf(6,1);+qpmax*ones(Nq,1)];

    % combine them
    Aeq = [A1eq;A2eq;A3eq];
    beq = [b1eq;b2eq;b3eq];

    %% get the solution
    % direct output are velocities
    options = optimoptions('fmincon','Algorithm','sqp','Display','none');
    [wsol,~] = fmincon(Jcost,winit,[],[],Aeq,beq,lb,ub,[],options);
    % integrate to get positions
    xsol = xst + wsol*dt;
end

