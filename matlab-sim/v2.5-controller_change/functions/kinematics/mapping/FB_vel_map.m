function [gen_vel] = FB_vel_map(eul,par_vel)
    %function maps parametrized velocities into generalised velocities
    Nq = size(par_vel,1)-6;
    
    %map
    [Efx] = get_FB_matrix(eul,Nq,'xyz');

    %generalized velocities
    gen_vel = Efx*par_vel;
end

