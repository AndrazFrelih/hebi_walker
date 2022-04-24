function [state_end] = gen_end_state(prev_state,Tsample,Tstate)
    state_end.id = 0;
    state_end.phase = 'd';
    state_end.Tstart = prev_state.Tstart + prev_state.dT;
    state_end.dT = Tstate;
    state_end.Npts = state_end.dT/Tsample;
    state_end.nstep = 0;
    state_end.shift_weight.val = 0;
    state_end.footstep.Lstart = prev_state.footstep.Lend;
    state_end.footstep.Rstart = prev_state.footstep.Rend;
    state_end.footstep.Lend = prev_state.footstep.Lend;
    state_end.footstep.Rend = prev_state.footstep.Rend;
    
    [tpts,tvec] = gen_tvec(state_end);
    
    state_end.footstep.Ltraj = generate_foot_traj(tpts,tvec,[state_end.footstep.Lend;0],[state_end.footstep.Lend;0],0);
    state_end.footstep.Rtraj = generate_foot_traj(tpts,tvec,[state_end.footstep.Rend;0],[state_end.footstep.Rend;0],0);
    state_end.zmp.start = prev_state.zmp.end;
    state_end.zmp.end = prev_state.zmp.end;
    state_end.zmp.traj.x = generate_lin_traj(tpts,tvec,state_end.zmp.start,state_end.zmp.end);
    state_end.zmp.traj.t = tvec;
    
    dLy = abs(state_end.footstep.Ltraj.x(2,1));
    state_end.shift_weight.traj.L = abs(state_end.zmp.traj.x(2,:)+state_end.footstep.Ltraj.x(2,:))/(2*dLy);
    state_end.shift_weight.traj.R = abs(state_end.zmp.traj.x(2,:)+state_end.footstep.Rtraj.x(2,:))/(2*dLy);

end

