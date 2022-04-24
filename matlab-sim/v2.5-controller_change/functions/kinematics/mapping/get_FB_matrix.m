function [Efx] = get_FB_matrix(eul,Nq,conv)
    if strcmp(conv,'xyz')
        B = B_XYZ(eul);
    else
        B = B_ZYX(eul);
    end

    
    %mapping
    Efx = [
        eye(3),      zeros(3),    zeros(3,Nq);
        zeros(3),    B,           zeros(3,Nq);
        zeros(Nq,3), zeros(Nq,3), eye(Nq);
    ];
end

