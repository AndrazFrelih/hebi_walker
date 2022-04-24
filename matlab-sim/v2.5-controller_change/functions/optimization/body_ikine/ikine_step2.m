function [xsol,wsol] = ikine_step2(meas, ctrl, xinit, winit, jacobians, ctrl_cfg, phase_data, cont_data)
    persistent int_pL int_pR int_com fd_ctrl_on phase_old init;
    if isempty(init)
       init = 1;
       int_pL = zeros(3,1);
       int_pR = zeros(3,1);
       int_com = zeros(3,1);
       fd_ctrl_on = zeros(2,1);
       phase_old = 0;
    end
    
    %% constants
    Ts = ctrl_cfg.par.Ts;
    Nq = ctrl_cfg.par.Nq;
    qpmax = ctrl_cfg.ik_opt.qp_lim;
    foot_data = ctrl_cfg.bal.foot_data;
    
    % foot sets
    Z1 = foot_data.dims(1:2)'/2 * 0.8;
    Z2 = foot_data.dims(1:2)'/2 * 0.9;
    Z3 = foot_data.dims(1:2)'/2 * 1.0;
    
    eul_conv = 'xyz';
    %% measured values
    % LIPM
    COM_m = meas.com;
    % TFs
    Hb_eeL_m = meas.Hb_eeL;
    Hb_eeR_m = meas.Hb_eeR;
    Hw_b_m = meas.Hw_b;
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
%     eulfb_m = rotm2eul(Rfb_m,eul_conv);
%     eulfb_m(3) = 0;
%     Rfb_m = eul2rotm(eulfb_m,eul_conv);
    
    qL_m = meas.qL;
    qR_m = meas.qR;
    q_m = [qL_m;qR_m];
    
    % contact states
    zmplocL_m = cont_data.L.zmp;
    zmplocR_m = cont_data.R.zmp;
    FL_m = cont_data.L.F;
    FR_m = cont_data.R.F;

    Fz_tot = 338;
%     contL = cont_data.L.cont;
%     contR = cont_data.R.cont;
    
    %% commanded values
    % LIPM
    COM_c = ctrl.com;
    COMp_c = ctrl.comp;
    
    % EE - obtained from the footstep traj. generator
    pL_c = ctrl.tL;
    RL_c = eye(3); %only for roll and pitch angles
    pLp_c = ctrl.vL;
    pR_c = ctrl.tR;
    RR_c = eye(3); %only for roll and pitch angles
    pRp_c = ctrl.vR;
    % robot states
    Rfb_c = eye(3); % trajectory generator
    q_c = ctrl.qold;
    
    %% phase data
    phase = phase_data.phase;
    if phase ~= phase_old
        ph_ch = 1;
        %reset integrators on the change of the phase
        int_com(3) = 0;
        int_pR = zeros(3,1);
        int_pL = zeros(3,1);
    else
        ph_ch = 0;
    end
    phase_old = phase;
    WshiftOff = 1/3; %1/3;
    WshiftLorg = phase_data.w(1) + WshiftOff;
    WshiftRorg = phase_data.w(2) + WshiftOff;
    Wshift = WshiftLorg + WshiftRorg;
    WshiftL = WshiftLorg/Wshift;
    WshiftR = WshiftRorg/Wshift;
    FL_c = Fz_tot * WshiftL;
    FR_c = Fz_tot * WshiftR;
    % swing leg criterion
    SwingL = ( phase == 1 ) && ( WshiftLorg < WshiftRorg );
    SwingR = ( phase == 1 ) && ( WshiftLorg > WshiftRorg );
    
    %% compute optimisation targets
    % (1) postural task - desired position of angles
    Ks_par = [1,1,1,1,1,1]/100;
    Ks = diag([Ks_par,Ks_par]);
    qerr = q_m - q_c;
    qp_d = -Ks*qerr;
    
    % (2) postural task - orientation of the floating base
    Kwfb_par = [50,50,50]*1;
    Kwfb = diag(Kwfb_par);
    wfb_d = Kwfb * get_skew_el(skew_mat2(Rfb_m*Rfb_c'));
    
    % (3) leg task - position and orientation of the foot
    Kvleg_par = [100,100,50];
    Kvleg_I_par = [20,20,0];
    Kwleg_par = [1,1,1]*2;
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

    %left leg gains
    KvlegL = Kvleg;
    KvlegL_I = Kvleg_I;
    KwlegL = Kwleg;
    
    if ~SwingL
        KvlegL_I = KwlegL*0;
    else
        KwlegL = KwlegL*30;
    end
    
    vL_fb = [
        -KvlegL * err_pL - KvlegL_I * int_pL;  %include the integrator
        -KwlegL * get_skew_el(skew_mat2(RL_m*RL_c'));
    ];

    vL_d = vL_ff + vL_fb;
    
    % right foot
    vR_ff = [
        pRp_c;
        zeros(3,1)
    ];

    err_pR = pR_m - pR_c;
    int_pR = int_pR + Ts * err_pR;

    %right leg gains
    KvlegR = Kvleg;
    KvlegR_I = Kvleg_I;
    KwlegR = Kwleg;
    
    if ~SwingR
        KvlegR_I = KwlegR*0;
    else
        KwlegR = KwlegR*30;
    end
    
    vR_fb = [
        -KvlegR * err_pR - KvlegR_I * int_pR;  %include the integrator
        -KwlegR * get_skew_el(skew_mat2(RR_m*RR_c'));
    ];
    
    vR_d = vR_ff + vR_fb;
    
    % foot difference control (DS)
    
    FLz_m = FL_m(3);
    FRz_m = FR_m(3);
    FLz_c = FL_c;
    FRz_c = FR_c;
    
    Kdfz = 0.0005;
    Tvdc = 1;
    v_dfz = Kdfz*((FLz_c - FRz_c)-(FLz_m - FRz_m)); % lift the foot under exess of pressure
    v_vdc = 1/Tvdc * ((pL_c(3) + pR_c(3))-(pL_m(3) + pR_m(3))); % prevent the foot positions from drifting off too far away
    vL_fdc = - 0.5 * v_dfz + 0.5 * v_vdc; %alredy implemented with foot position feedback term
    vR_fdc = + 0.5 * v_dfz + 0.5 * v_vdc;
    
    if phase == 0 % only use in DS phase
        vL_d(3) = vL_d(3) + vL_fdc;
        vR_d(3) = vR_d(3) + vR_fdc;
    end
    
    % foot damping
    fd_act = 1;
    if fd_act
        [w_fd_ctrl_L, fd_ctrl_on(1)] = foot_damping_ctrl(zmplocL_m,Z1,Z2,Z3,fd_ctrl_on(1)); 
        [w_fd_ctrl_R, fd_ctrl_on(2)] = foot_damping_ctrl(zmplocR_m,Z1,Z2,Z3,fd_ctrl_on(2));
        if fd_ctrl_on(1) && ~SwingL
            vL_d(4:6) = 30*vL_d(4:6);
        end

        if fd_ctrl_on(2) && ~SwingR
            vR_d(4:6) = 30*vR_d(4:6);
        end
    end
    
    % COM task
    Kcom_par = [1,1,5];
    Kcom_I_par = [1,1,10];
    
    Kcom = diag(Kcom_par);
    Kcom_I = diag(Kcom_I_par);
    
    err_com = COM_m - COM_c;
    int_com = int_com + Ts * err_com;
    vcom_d = COMp_c - Kcom * err_com - Kcom_I * int_com; %include the integrator
    
    %% compute jacobians
    Jw_com = jacobians.Jw_com;
    Jw_eeL = jacobians.Jw_eeL;
    Jw_eeR = jacobians.Jw_eeR;
    Jw_b = jacobians.Jw_b;
    
    %% setup of Px, qx and Ax matrices and bounds
    % Px
    R1x = Jw_b(4:6,:);
    W1x = eye(min(size(R1x)));
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

