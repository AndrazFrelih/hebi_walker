function [pg_traj] = create_traj(state_list, Nsamples)
    zmp_val = zeros(2,Nsamples);
    lfoot_val = zeros(3,Nsamples);
    rfoot_val = zeros(3,Nsamples);
    lfoot_val_p = zeros(3,Nsamples);
    rfoot_val_p = zeros(3,Nsamples);
    phase = zeros(1,Nsamples);
    weight_shift = zeros(2,Nsamples);
    Nstates = max(size(state_list));
    for i=1:Nstates
        if i==1
            ist = 1;
            ien = state_list{i}.Npts;
        else
            ist = ien+1;
            ien = ien+state_list{i}.Npts;
        end
        zmp_val(:,ist:ien) = state_list{i}.zmp.traj.x;
        lfoot_val(:,ist:ien) = state_list{i}.footstep.Ltraj.x;
        rfoot_val(:,ist:ien) = state_list{i}.footstep.Rtraj.x;
        lfoot_val_p(:,ist:ien) = state_list{i}.footstep.Ltraj.xp;
        rfoot_val_p(:,ist:ien) = state_list{i}.footstep.Rtraj.xp;
        if(state_list{i}.phase == 's')
            phase(ist:ien) = ones(1,state_list{i}.Npts);
        else
            phase(ist:ien) = zeros(1,state_list{i}.Npts);
        end
        weight_shift(1,ist:ien) = state_list{i}.shift_weight.traj.L;
        weight_shift(2,ist:ien) = state_list{i}.shift_weight.traj.R;
    end
    
    pg_traj.zmp = zmp_val;
    pg_traj.w = weight_shift;
    pg_traj.foot.xL = lfoot_val;
    pg_traj.foot.xR = rfoot_val;
    pg_traj.foot.xLp = lfoot_val_p;
    pg_traj.foot.xRp = rfoot_val_p;
    pg_traj.phase = phase;
end

