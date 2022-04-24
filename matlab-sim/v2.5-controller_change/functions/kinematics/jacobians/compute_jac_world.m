function [Jlist_world] = compute_jac_world(Hwb,Jlist_base,Hlist)
    Rwb = Hwb(1:3,1:3);
    Nlist = size(Jlist_base,1);
    ind_l = 3;
    Ndof = 6;
    Njorg = size(Jlist_base,ind_l);
    Nvar = 6+Njorg;
    
    Jlist_world = zeros(Nlist,Ndof,Nvar);
    
    for i=1:Nlist
        Hb_i = reshape(Hlist(i,:,:),4,4);
        vec = Hb_i(1:3,4);

        Jbase = [
            eye(3),   -Rwb*skew_mat(vec);
            zeros(3),  Rwb;
        ];

        if i~=1
            Jmainleg = [
                Rwb*reshape(Jlist_base(i,1:3,:),Ndof/2,Njorg);
                Rwb*reshape(Jlist_base(i,4:6,:),Ndof/2,Njorg);
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

