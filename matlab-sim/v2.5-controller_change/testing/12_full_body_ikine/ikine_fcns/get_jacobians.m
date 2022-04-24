function [Jw_com, Jw_eeL, Jw_eeR, Jw_b] = get_jacobians(X,mlist)
    x = X(1:3);
    e = X(4:6);
    qL = X(7:12);
    qR = X(13:18);

    Nq = 12;

    % base to world
    Hw_b = Hbase_in([x;e]);

    % get abs joint transforms and com transforms
    [Hb_com,Hb_L,Hb_R] = H_com_list(qL,qR);
    Hb_i = zeros(Nq+2,4,4); 
    Hb_i(1:end/2,:,:) = Hb_L;
    Hb_i(end/2+1:end,:,:) = Hb_R;

    %% create jacobians
    % get com->base jacobians 
    Jb_comi = compute_com_jac_base(Nq,Hb_i,Hb_com);

    % get ee->base jaccobian
    Jb_eeL = compute_jacobian(Hb_L,7);
    Jb_eeR = compute_jacobian(Hb_R,7);

    %% transform them to world jacobians and add a jacobian for the base
    frame_flag = 'b';
    % get com->world jacobians 
    Jw_comi = compute_jac_world2(Hw_b,Jb_comi,Hb_i,frame_flag);

    % get ee->world jacobian
    % left
    Hb_eeL = reshape(Hb_L(end,:,:),4,4);
    Jw_eeL_g = compute_jac_world_single2(Hw_b, Jb_eeL, Hb_eeL, 'l', frame_flag);
    % right
    Hb_eeR = reshape(Hb_R(end,:,:),4,4);
    Jw_eeR_g = compute_jac_world_single2(Hw_b, Jb_eeR, Hb_eeR, 'r', frame_flag);

    % get base->world jacobian
    Jw_b_g = compute_jac_world_single2(Hw_b, zeros(6,Nq/2), eye(4), 'b', frame_flag);
    
    %% compute com jacobian
    Jw_com_g = compute_centroidal_jac(Jw_comi,mlist);
    
    %% transform jacobians so that they are consistent with the configuration
    % account for the configuration dependent differences in mapping
    % between generalised and parametrised velocities
    [Efx] = get_FB_matrix(e,Nq,'xyz');

    Jw_eeL = Jw_eeL_g * Efx;
    Jw_eeR = Jw_eeR_g * Efx;
    Jw_b   = Jw_b_g   * Efx;
    Jw_com = Jw_com_g * Efx;
end

