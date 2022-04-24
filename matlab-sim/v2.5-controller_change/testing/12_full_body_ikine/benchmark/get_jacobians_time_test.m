function [Jw_com, Jw_eeL, Jw_eeR, Jw_b] = get_jacobians_time_test(X,mlist)
    x = X(1:3);
    e = X(4:6);
    qL = X(7:12);
    qR = X(13:18);

    Nq = 12;

    disp("T1:");
    tic
    % base to world
    Hw_b = Hbase_in([x;e]);
    toc

    % get abs joint transforms and com transforms
    
    disp("T2:");
    tic
    [Hb_com,Hb_L,Hb_R] = H_com_list(qL,qR);
    toc

    disp("T3:");
    tic
    Hb_i = zeros(Nq+2,4,4); 
    Hb_i(1:end/2,:,:) = Hb_L;
    Hb_i(end/2+1:end,:,:) = Hb_R;
    toc %is fast, but if it has to be done in each iteration, then it is pretty slow

    %% create jacobians
    % get com->base jacobians 
    disp("T4:");
    tic
    Jb_comi = compute_com_jac_base(Nq,Hb_i,Hb_com);
    toc %slowest part of the code!! -> can only be improved by using a different language (with pointers)

    % get ee->base jaccobian
    disp("T5:");
    tic
    Jb_eeL = compute_jacobian(Hb_L,7);
    Jb_eeR = compute_jacobian(Hb_R,7);
    toc %very fast (for one on avg 101us)

    %% transform them to world jacobians and add a jacobian for the base
    frame_flag = 'b';
    % get com->world jacobians 
    disp("T6:");
    tic
    Jw_comi = compute_jac_world(Hw_b,Jb_comi,Hb_i);
    %Jw_comi = compute_jac_world2(Hw_b,Jb_comi,Hb_i,frame_flag);
    toc %quite fast

    % get ee->world jacobian
    % left
    disp("T7:");
    tic
    Hb_eeL = reshape(Hb_L(end,:,:),4,4);
    Jw_eeL_g = compute_jac_world_single2(Hw_b, Jb_eeL, Hb_eeL, 'l', frame_flag);
    % right
    Hb_eeR = reshape(Hb_R(end,:,:),4,4);
    Jw_eeR_g = compute_jac_world_single2(Hw_b, Jb_eeR, Hb_eeR, 'r', frame_flag);
    toc %very fast (for one on avg 110us)

    % get base->world jacobian
    disp("T8:");
    tic
    Jw_b_g = compute_jac_world_single2(Hw_b, zeros(6,Nq/2), eye(4), 'b', frame_flag);
    toc %very fast

    %% compute com jacobian
    disp("T9:");
    tic
    Jw_com_g = compute_centroidal_jac(Jw_comi,mlist);
    toc %super fast

    %% transform jacobians so that they are consistent with the configuration
    % account for the configuration dependent differences in mapping
    % between generalised and parametrised velocities
    disp("T10:"); 
    tic
    [Efx] = get_FB_matrix(e,Nq,'xyz');
    toc %very fast
    
    disp("T11:"); 
    tic
    Jw_eeL = Jw_eeL_g * Efx;
    Jw_eeR = Jw_eeR_g * Efx;
    Jw_b   = Jw_b_g   * Efx;
    Jw_com = Jw_com_g * Efx;
    toc %super fast
end

