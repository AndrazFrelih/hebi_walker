function [state,Nsamples] = pregen_pattern(Nstep, Times, app_type)
%step characteristics
stepX = 0.04;
stepZ = 0.02;

%times
Tsample = Times.Tsample;
Tini = Times.Tini;
Tss = Times.Tss;
Tds = Times.Tds;
Tfin = Times.Tfin;

if  app_type==0 || app_type==1 
    %only shift weight
    Lx = 0; 
    Lz = 0;
elseif app_type == 2 || app_type==-1 
    %step in place
    Lx = 0; 
    Lz = stepZ;
elseif app_type == 3
    %move 
    Lx = stepX; 
    Lz = stepZ;   
end
Ly = 0.25;
dLy = Ly/2;

%pattern for x
StepX = Lx*ones(1,Nstep);
%pattern for y
StepY = Ly*ones(1,Nstep);
StepY(2:2:end) = StepY(2:2:end)*-1;
%pattern for yaw
StepYaw = zeros(1,Nstep);

%generate a pattern for single supports
PSStraj = zeros(3,Nstep);
for i=1:Nstep
    Rz3d = RotZ(StepYaw(i));
    Rz2d = Rz3d(1:2,1:2);
    if i>1
        PSStraj(1:2,i) = PSStraj(1:2,i-1) + Rz2d*[StepX(i);StepY(i)];
    else
        PSStraj(1:2,i) = Rz2d*[0;StepY(i)/2];
    end
    PSStraj(3,i) = StepYaw(i);
end
    
%create a sequence of ZMPs (zmp values at a transition between states)
press_seq_half = sign(PSStraj(2,:));
press_seq = zeros(1,max(size(press_seq_half))*2);
press_seq(1:2:end) = press_seq_half;
press_seq(2:2:end) = press_seq_half;
press_seq_full = [0,press_seq,0];

%get phases 
phase = press_seq_full(2:end)-press_seq_full(1:end-1);

%how to shift weight (-1 to left leg, +1 to right leg, 0 to middle)
shift_weight = phase + press_seq_full(1:end-1);


state = cell(2*Nstep+1,1);

Nsamples = 0;
Nstates = max(size(phase));
for i=1:Nstates
    %enumerate the state
    state{i}.id = i;
    %assign a phase to the state
    if phase(i)==0
        state{i}.phase = 's';
        state{i}.dT = Tss;
    else
        state{i}.phase = 'd';
        if i==1
            state{i}.dT = Tds/2;
        else
            state{i}.dT = Tds;
        end
    end
    
    %compute the number of time instants for this state
    state{i}.Npts = state{i}.dT/Tsample;
    Nsamples = Nsamples + state{i}.Npts; %calculate the length of the trajectory
    
    %compute the total time that has been spent to the point of this step
    if i==1
        state{i}.Tstart = Tini;
    else
        state{i}.Tstart = state{i-1}.Tstart + state{i-1}.dT;
    end
    
    %store how many steps will have already been made after this phase
    if i==1
        state{i}.nstep = 0;
    else
        if strcmp(state{i-1}.phase,'d')
            state{i}.nstep = state{i-1}.nstep+1;
        else
            state{i}.nstep = state{i-1}.nstep;
        end
    end
    
    %where should the weight be shifted to during the phase
    state{i}.shift_weight.val = shift_weight(i);
    
    %store step positions
    if strcmp(state{i}.phase,'d')
        %double support phase
        if i==1
            state{i}.footstep.Lstart = [PSStraj(1,1);+PSStraj(2,1)];
            state{i}.footstep.Rstart = [PSStraj(1,1);-PSStraj(2,1)];
        else
            state{i}.footstep.Lstart = state{i-1}.footstep.Lend;
            state{i}.footstep.Rstart = state{i-1}.footstep.Rend;
        end
        state{i}.footstep.Lend = state{i}.footstep.Lstart;
        state{i}.footstep.Rend = state{i}.footstep.Rstart;
    else
        %single support phase
        state{i}.footstep.Lstart = state{i-1}.footstep.Lend;
        state{i}.footstep.Rstart = state{i-1}.footstep.Rend;
        
        if state{i}.nstep<Nstep
            %is not the last step
            if state{i}.shift_weight.val>0
                %left foot stays fixed through this SS phase
                state{i}.footstep.Lend = state{i}.footstep.Lstart;
                state{i}.footstep.Rend = PSStraj(1:2,state{i}.nstep+1);
            else
                %right foot stays fixed through this SS phase    
                state{i}.footstep.Rend = state{i}.footstep.Rstart;
                state{i}.footstep.Lend = PSStraj(1:2,state{i}.nstep+1);
            end
        else
            %last step requires special treatment (only a half step)
            if state{i}.shift_weight.val>0
                %left foot stays fixed through this SS phase
                state{i}.footstep.Lend = state{i}.footstep.Lstart;
                state{i}.footstep.Rend = [state{i}.footstep.Lend(1);-state{i}.footstep.Lend(2)];
            else
                %right foot stays fixed through this SS phase    
                state{i}.footstep.Rend = state{i}.footstep.Rstart;
                state{i}.footstep.Lend = [state{i}.footstep.Rend(1);-state{i}.footstep.Rend(2)];
            end
        end
    end
    
    %compute trajectories 
    [tpts,tvec] = gen_tvec(state{i});
    
    %feet trajectories
    Xst_L = [state{i}.footstep.Lstart;0];
    Xen_L = [state{i}.footstep.Lend;0];
    Xst_R = [state{i}.footstep.Rstart;0];
    Xen_R = [state{i}.footstep.Rend;0]; 
    
    if strcmp(state{i}.phase,'s')
        %single support phase - one of the feet moves
        if state{i}.shift_weight.val > 0
            %left leg is the support leg
            state{i}.footstep.Ltraj = generate_foot_traj(tpts,tvec,Xst_L,Xen_L,0);
            %right leg is the flight leg 
            state{i}.footstep.Rtraj = generate_foot_traj(tpts,tvec,Xst_R,Xen_R,Lz);
        else
            %right leg is the support leg
            state{i}.footstep.Rtraj = generate_foot_traj(tpts,tvec,Xst_R,Xen_R,0);
            %left leg is the flight leg
            state{i}.footstep.Ltraj = generate_foot_traj(tpts,tvec,Xst_L,Xen_L,Lz);
        end
    else
        %double support phase - feet stay in the same position
        state{i}.footstep.Ltraj = generate_foot_traj(tpts,tvec,Xst_L,Xen_L,0);
        state{i}.footstep.Rtraj = generate_foot_traj(tpts,tvec,Xst_R,Xen_R,0);
    end
    
    %zmp intermediate positions
    if i==1
        %first DS requires special treatment
        %start with the zmp in the middle
        state{i}.zmp.start = (state{i}.footstep.Lstart + state{i}.footstep.Rstart)/2;
        if state{i}.shift_weight.val>0
            state{i}.zmp.end = state{i}.footstep.Lend;
        else
            state{i}.zmp.end = state{i}.footstep.Rend;
        end
    elseif i==Nstates
        %last DS requires special treatment
        if state{i-1}.shift_weight.val>0
            state{i}.zmp.start = state{i-1}.footstep.Lend;
        else
            state{i}.zmp.start = state{i-1}.footstep.Rend;
        end
        %end with the zmp in the middle
        state{i}.zmp.end = (state{i}.footstep.Lstart + state{i}.footstep.Rstart)/2;
    else
        %starting zmp (check previous location of the weight)
        if state{i-1}.shift_weight.val>0
            state{i}.zmp.start = state{i-1}.footstep.Lend;
        elseif state{i-1}.shift_weight.val<0
            state{i}.zmp.start = state{i-1}.footstep.Rend;
        else
            disp("Bad programming");
        end
        %ending zmp (check current weight location)
        if state{i}.shift_weight.val>0
            state{i}.zmp.end = state{i}.footstep.Lstart;
        elseif state{i}.shift_weight.val<0
            state{i}.zmp.end = state{i}.footstep.Rstart;
        else
            disp("Bad programming");
        end
    end
    %compute zmp trajectory
    state{i}.zmp.traj.x = generate_lin_traj(tpts,tvec,state{i}.zmp.start,state{i}.zmp.end);
    state{i}.zmp.traj.t = tvec;
    
    %now compute the weight shifting trajectory (needed for wrench
    %distribution)
    state{i}.shift_weight.traj.L = abs(state{i}.zmp.traj.x(2,:)+state{i}.footstep.Ltraj.x(2,:))/(2*dLy);
    state{i}.shift_weight.traj.R = abs(state{i}.zmp.traj.x(2,:)+state{i}.footstep.Rtraj.x(2,:))/(2*dLy);
    
    %the required weight shifting is too large, try reduce it (impotant
    %that it is done after the weight shifting trajectory computation)
    state{i}.zmp.traj.x(2,:) = state{i}.zmp.traj.x(2,:)*0.5;
end


%% append a state (has to be added due to preview induced latency)
[state_end] = gen_end_state(state{end},Tsample,Tini+Tfin);
Nsamples = Nsamples + state_end.Npts;
Nstates = Nstates + 1;
state = [state;{state_end}];

end

