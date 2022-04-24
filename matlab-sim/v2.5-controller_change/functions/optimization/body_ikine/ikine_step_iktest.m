function [xsol,wsol] = ikine_step_iktest(meas, ctrl, xinit, winit, jacobians, ctrl_cfg, phase_data, cont_data)
    persistent int_pL int_pR  init;
    if isempty(init)
       init = 1;
       int_pL = zeros(3,1);
       int_pR = zeros(3,1);
    end
    

    Ts = ctrl_cfg.par.Ts;
    Nq = ctrl_cfg.par.Nq;
    qpmax = ctrl_cfg.ik_opt.qp_lim;

    eul_conv = 'xyz';
    %% measured values
    
    add_offs = [0;0;0.000];
    
    % TFs
    Hb_eeL_m = meas.Hb_eeL;
    Hb_eeR_m = meas.Hb_eeR;
    Hw_b_m = ctrl_cfg.fb_est.init.Hw_b;
    Hw_eeL_m = Hw_b_m * Hb_eeL_m;
    Hw_eeR_m = Hw_b_m * Hb_eeR_m;
    
    % EE - obtained from FKine
    pL_m = Hw_eeL_m(1:3,4);
    RL_m_withYaw = Hw_eeL_m(1:3,1:3); % yaw angle is not important (remove it)
    eulL_m = rotm2eul(RL_m_withYaw,eul_conv);
    eulL_m(3) = 0;
    RL_m = eul2rotm(eulL_m,eul_conv);
    
    pR_m = Hw_eeR_m(1:3,4);
    RR_m_withYaw = Hw_eeR_m(1:3,1:3); % yaw angle is not important (remove it)
    eulR_m = rotm2eul(RR_m_withYaw,eul_conv);
    eulR_m(3) = 0;
    RR_m = eul2rotm(eulR_m,eul_conv);
    
    % robot states
    Rfb_m = Hw_b_m(1:3,1:3);
    qL_m = meas.qL;
    qR_m = meas.qR;
    q_m = [qL_m;qR_m];
    
    
    %% commanded values
    % EE - obtained from the footstep traj. generator
    pL_c = ctrl.tL  + add_offs;
    RL_c = eye(3); %only for roll and pitch angles
    pLp_c = ctrl.vL;
    pR_c = ctrl.tR  + add_offs;
    RR_c = eye(3); %only for roll and pitch angles
    pRp_c = ctrl.vR;
    % robot states
    Rfb_c = eye(3); % trajectory generator
    q_c = ctrl.qold;
    
    
    %% compute optimisation targets
    % (1) postural task - desired position of angles
    Ks_par = [1,1,1,1,1,1]/100;
    Ks = diag([Ks_par,Ks_par]);
    qerr = q_m - q_c;
    qp_d = -Ks*qerr;
    
    
    % (3) leg task - position and orientation of the foot
    Kvleg_par = [1,1,1]*100;
    Kvleg_I_par = [1,1,1]*20;
    Kwleg_par = [1,1,1]*100;
    Kvleg = diag(Kvleg_par);
    Kvleg_I = diag(Kvleg_I_par);
    Kwleg = diag(Kwleg_par);
    % left foot
    vL_ff = [
        pLp_c;
        zeros(3,1)
    ];
    
    err_pL = pL_m - pL_c;
    int_pL = int_pL + Ts * err_pL;

    vL_fb = [
        -Kvleg * err_pL - Kvleg_I * int_pL;  %include the integrator
        -Kwleg * get_skew_el(skew_mat2(RL_m*RL_c'));
    ];    
    vL_d = vL_ff + vL_fb;
    
    % right foot
    vR_ff = [
        pRp_c;
        zeros(3,1)
    ];

    err_pR = pR_m - pR_c;
    int_pR = int_pR + Ts * err_pR;

    vR_fb = [
        -Kvleg * err_pR - Kvleg_I * int_pR;  %include the integrator
        -Kwleg * get_skew_el(skew_mat2(RR_m*RR_c'));
    ];
    vR_d = vR_ff + vR_fb;


    %% compute jacobians
    Jw_eeL = jacobians.Jw_eeL;
    Jw_eeR = jacobians.Jw_eeR;
    Jw_b = jacobians.Jw_b;
    
    %% setup of Px, qx and Ax matrices and bounds
    % Px

    R2x = [
        zeros(Nq,6), eye(Nq);
    ];
    W2x = eye(min(size(R2x))); %do not include the task as here measurements are not available
    s2x = qp_d;
    P2x = R2x'*W2x*R2x;
    q2x = R2x'*W2x*s2x;

    Px = + P2x;
    qx = + q2x;

    % cost function
    Jcost = @(w)w'*Px*w+qx'*w;

    % Aeq and beq formulation
    A1eq = Jw_b(1:6,:);
    b1eq = zeros(6,1);
    
    A2eq = Jw_eeL;
    b2eq = vL_d;

    A3eq = Jw_eeR;
    b3eq = vR_d;
        
    % bounds
    %qpmax = 10;
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
    xsol = xinit + wsol*Ts;
end

