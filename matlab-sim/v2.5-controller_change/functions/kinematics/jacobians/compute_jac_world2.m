function [Jlist_world] = compute_jac_world2(Hwb,Jlist_b_q,Hlist,frame_flag)
    Rwb = Hwb(1:3,1:3);
    Nlist = size(Jlist_b_q,1);
    ind_l = 3;
    Ndof = 6;
    Njorg = size(Jlist_b_q,ind_l);
    Nvar = 6+Njorg;
    
    Jlist_world = zeros(Nlist,Ndof,Nvar);
    
    % get the mappings for base velocities (jacob matrices depend on the frame
    % in which velocities are available)
    
    for i=1:Nlist
        Hb_i = reshape(Hlist(i,:,:),4,4);
        vec = Hb_i(1:3,4);

        [Jbase] = get_base_jac(Rwb,vec,frame_flag);

        if i~=1
            Jmainleg = [
                Rwb*reshape(Jlist_b_q(i,1:3,:),Ndof/2,Njorg);
                Rwb*reshape(Jlist_b_q(i,4:6,:),Ndof/2,Njorg);
            ];
            Jlist_world(i,:,:) = [
                Jbase,Jmainleg
            ];
        else    
            Jlist_world(i,:,:) = [
                Jbase,zeros(Ndof,Njorg); %joints do not influence the kinetic velocity of the base
            ];
        end
    end

end

