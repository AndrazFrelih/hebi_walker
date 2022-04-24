function [TS] = gen_timeseries(state,tvec)
    Nstates = size(state,1);
    Ntot = get_Ntotal(state);
    Ndata = 39;
    
    Data = zeros(Ndata,Ntot);
    
    ien = 0;
    for i=1:Nstates
        ist = ien+1;
        ien = ien+state{i}.Npts;
        %encoding of single & double support
        if state{i}.phase == 'd'
            Data(1,ist:ien) = zeros(1,state{i}.Npts);
        elseif state{i}.phase == 's'
            Data(1,ist:ien) = ones(1,state{i}.Npts);
        end
        
        %weight shift factors
        Data(2,ist:ien) = state{i}.shift_weight.traj.L;
        Data(3,ist:ien) = state{i}.shift_weight.traj.R;
        
        %desired zmp
        Data(4:5,ist:ien) = state{i}.zmp.traj.x;
        
        %desired com position
        Data(6:7,ist:ien) = state{i}.com.x(1:2,:);
        
        %desired com velocity
        Data(8:9,ist:ien) = state{i}.com.xp(1:2,:);
        
        %desired left and right joint angle values
        Data(10:15,ist:ien) = state{i}.joint.q.L;
        Data(16:21,ist:ien) = state{i}.joint.q.R;
        
        %desired left and right joint velocity values
        Data(22:27,ist:ien) = state{i}.joint.qp.L;
        Data(28:33,ist:ien) = state{i}.joint.qp.R;
        
        %desired footstep position
        Data(34:36,ist:ien) = state{i}.footstep.Ltraj.x;
        Data(37:39,ist:ien) = state{i}.footstep.Rtraj.x;
        
        %desired footstep velocity
        Data(40:42,ist:ien) = state{i}.footstep.Ltraj.xp;
        Data(43:45,ist:ien) = state{i}.footstep.Rtraj.xp;
    end
    
    TS = timeseries(Data',tvec');
end

