function [J_wrld] = compute_jac_world_single2(Hwb,Jb_ee,Hbi,side,frame_flag)
    %% INPUTS
    % Hwb        ->  transformation between base and world
    % J_base     ->  jacobian between the frame represented by Hbi and base. 
    %                It is of the size 6 x 6 (single leg jacobian)
    % Hbi        ->  transformation between the observed frame and base
    % side       ->  is it left of right leg
    % frame_flag ->  gives information about the frame of reference for
    %                   the available velocities (w - world, b - base)
    
    %% OUTPUTS
    % J_wrld     ->  size 6 x 18 (base & both legs, one of the legs has no
    %                influence and it's entries are consequently set to zero  
    
    %% FUNCTION
    Rwb = Hwb(1:3,1:3);
    vec = Hbi(1:3,4);
    
    % get the mappings for base velocities (matrices depend on the frame
    % in which velocities are available)
    [Jbase] = get_base_jac(Rwb,vec,frame_flag);

    Jmainleg = [
        Rwb*Jb_ee(1:3,:);
        Rwb*Jb_ee(4:6,:);
    ];

    if side=='l'
        J_wrld = [
            Jbase,Jmainleg,zeros(size(Jmainleg));
        ];
    elseif side=='r'
        J_wrld = [
            Jbase,zeros(size(Jmainleg)),Jmainleg;
        ];
    else
        J_wrld = [
            Jbase,zeros(size(Jmainleg)),zeros(size(Jmainleg));
        ];
    end
end

