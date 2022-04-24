function [Jw_b] = get_base_jac(Rwb,vec,frame_flag)
    if frame_flag(1)=='w'
        %jacobian mapping if the angular velocity is known in the inertial
        %frame I (world frame) -> IwIB 
        Jw_b = [
            eye(3),   -Rwb*skew_mat(vec)*Rwb';
            zeros(3),  eye(3);
        ];
    else
        %jacobian mapping if the angular velocity is known in the base
        %frame B -> BwIB 
        Jw_b = [
            eye(3),   -Rwb*skew_mat(vec);
            zeros(3),  Rwb;
        ];
    end
end

