function [state_ini] = gen_init_state(next_state,Tsample)
    state_ini.id = 0;
    state_ini.phase = 'd';
    state_ini.Tstart = 0;
    state_ini.dT = next_state.Tstart;
    state_ini.Npts = state_ini.dT/Tsample;
    state_ini.nstep = 0;
    state_ini.shift_weight.val = 0;
    state_ini.footstep.Lstart = next_state.footstep.Lstart;
    state_ini.footstep.Rstart = next_state.footstep.Rstart;
    state_ini.footstep.Lend = next_state.footstep.Lstart;
    state_ini.footstep.Rend = next_state.footstep.Rstart;
    
    [tpts,tvec] = gen_tvec(state_ini);
    
    state_ini.footstep.Ltraj = generate_foot_traj(tpts,tvec,[state_ini.footstep.Lstart;0],[state_ini.footstep.Lend;0],0);
    state_ini.footstep.Rtraj = generate_foot_traj(tpts,tvec,[state_ini.footstep.Rstart;0],[state_ini.footstep.Rend;0],0);
    state_ini.zmp.start = next_state.zmp.start;
    state_ini.zmp.end = next_state.zmp.start;
    state_ini.zmp.traj.x = generate_lin_traj(tpts,tvec,state_ini.zmp.start,state_ini.zmp.end);
    state_ini.zmp.traj.t = tvec;
    
    dLy = abs(state_ini.footstep.Ltraj.x(2,1));
    state_ini.shift_weight.traj.L = abs(state_ini.zmp.traj.x(2,:)+state_ini.footstep.Ltraj.x(2,:))/(2*dLy);
    state_ini.shift_weight.traj.R = abs(state_ini.zmp.traj.x(2,:)+state_ini.footstep.Rtraj.x(2,:))/(2*dLy);

end

