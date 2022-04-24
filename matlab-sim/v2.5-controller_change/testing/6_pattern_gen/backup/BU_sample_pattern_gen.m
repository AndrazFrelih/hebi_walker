Nstep = 3;
Nstep_ada = Nstep;

Zdes = ctrl_config.com_height;
Tsample = 1/ctrl_config.fctrl;

%type of generated trajectory
app_type = 1;
if app_type==1
   %move 
   Lx = 0.1; 
   Lz = 0.03;
elseif app_type == 2
   %step in place
   Lx = 0; 
   Lz = 0.03;
else
   %only shift weight
   Lx = 0; 
   Lz = 0;
end
Ly = (abs(Tf_ee_L(2,4))+abs(Tf_ee_R(2,4)));


%pattern for x
StepX = Lx*ones(1,Nstep_ada);
%pattern for y
StepY = Ly*ones(1,Nstep_ada);
StepY(2:2:end) = StepY(2:2:end)*-1;
%pattern for yaw
StepYaw = zeros(1,Nstep_ada);

%generate a pattern for single supports
PSStraj = zeros(3,Nstep_ada);
for i=1:Nstep_ada
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

Tss = 0.5;
Tds = 1;
Tini = 1;
Tfin = 2;

state = cell(2*Nstep_ada+1,1);

T_ph = [Tds,Tss];
s_ph = ["DS","SS"];

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
        state{i}.dT = Tds;
    end
    
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
    state{i}.shift_weight = shift_weight(i);
    
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
        
        if state{i}.nstep<Nstep_ada
            %is not the last step
            if state{i}.shift_weight>0
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
            if state{i}.shift_weight>0
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
    state{i}.Npts = state{i}.dT/Tsample;
    Nsamples = Nsamples + state{i}.Npts; %calculate the length of the trajectory
    tpts = [0,state{i}.dT];
    tvec = linspace(tpts(1),tpts(2),state{i}.Npts);
    
    %feet trajectories
    Xst_L = [state{i}.footstep.Lstart;0];
    Xen_L = [state{i}.footstep.Lend;0];
    Xst_R = [state{i}.footstep.Rstart;0];
    Xen_R = [state{i}.footstep.Rend;0]; 
    
    if strcmp(state{i}.phase,'s')
        %single support phase - one of the feet moves
        if state{i}.shift_weight > 0
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
        if state{i}.shift_weight>0
            state{i}.zmp.end = state{i}.footstep.Lend;
        else
            state{i}.zmp.end = state{i}.footstep.Rend;
        end
    elseif i==Nstates
        %last DS requires special treatment
        if state{i-1}.shift_weight>0
            state{i}.zmp.start = state{i-1}.footstep.Lend;
        else
            state{i}.zmp.start = state{i-1}.footstep.Rend;
        end
        %end with the zmp in the middle
        state{i}.zmp.end = (state{i}.footstep.Lstart + state{i}.footstep.Rstart)/2;
    else
        %starting zmp (check previous location of the weight)
        if state{i-1}.shift_weight>0
            state{i}.zmp.start = state{i-1}.footstep.Lend;
        elseif state{i-1}.shift_weight<0
            state{i}.zmp.start = state{i-1}.footstep.Rend;
        else
            disp("Bad programming");
        end
        %ending zmp (check current weight location)
        if state{i}.shift_weight>0
            state{i}.zmp.end = state{i}.footstep.Lstart;
        elseif state{i}.shift_weight<0
            state{i}.zmp.end = state{i}.footstep.Rstart;
        else
            disp("Bad programming");
        end
    end
    %compute zmp trajectory
    state{i}.zmp.traj.x = generate_lin_traj(tpts,tvec,state{i}.zmp.start,state{i}.zmp.end);
    state{i}.zmp.traj.t = tvec;
end

%calculate parameters for the preview controller
Tsim = state{Nstates}.Tstart + state{Nstates}.dT + Tfin;
Tpre = 1; %preview time (has to be equal to / smaller than Tini)
Pctrl = calculate_preview(Zdes, Tsample, Tpre, Tsim);

%get full zmp trajectory
zmp_val = zeros(2,Nsamples);
full_time = zeros(1,Nsamples);
lfoot_val = zeros(3,Nsamples);
rfoot_val = zeros(3,Nsamples);
for i=1:Nstates
    if i==1
        ist = 1;
        ien = state{i}.Npts;
        ttmp = state{i}.zmp.traj.t;
    else
        ist = ien+1;
        ien = ien+state{i}.Npts;
        ttmp = state{i}.zmp.traj.t+state{i}.Tstart;
    end
    zmp_val(:,ist:ien) = state{i}.zmp.traj.x;
    full_time(ist:ien) = ttmp;
    lfoot_val(:,ist:ien) = state{i}.footstep.Ltraj.x;
    rfoot_val(:,ist:ien) = state{i}.footstep.Rtraj.x;
end

figure(1);
plot(full_time,zmp_val);
grid on
figure(2);
plot(full_time,lfoot_val);
grid on
figure(3);
plot(full_time,rfoot_val);
grid on

%simulation for x-com
sim_input = timeseries(zmp_val(1,:)',full_time');
sim_X = sim('com_trajectory_gen','CaptureErrors','on');

%simulation for y-com
sim_input = timeseries(zmp_val(2,:)',full_time');
sim_Y = sim('com_trajectory_gen','CaptureErrors','on');

%group the results into the structure as well

%% calculate the initial pose of the robot:
% first a statically stable initial state is computed
C_des = [
    sim_X.C.Data(1);
    sim_Y.C.Data(1);
    Zdes+0.05;
];

xshift=0;
% left foot task
XLdes = [
    xshift+lfoot_val(1,1); %initial X-position for center of foot
    lfoot_val(2,1); %initial Y-position for center of foot
    zeros(4,1);     %initial Z-position for all vertices
];

% right foot task
XRdes = [
    xshift+rfoot_val(1,1); %initial X-position for center of foot
    rfoot_val(2,1); %initial Y-position for center of foot
    zeros(4,1);     %initial Z-position for all vertices
];

% initial search value for the base position and joint angles
Xbase = [0;0;ctrl_config.base_height];
Xini = [Xbase;Q_init];
Nz = size(Xini,1); %dimensionality of the initial problem
Nq = size(Q_init,1); %dimensionality of the joint space

% define fixed feet transforms
H_foot_fix = zeros(4,4,4);
H_foot_fix(1,:,:) = Transl([foot.corners(1,:),0]'); %back left
H_foot_fix(2,:,:) = Transl([foot.corners(2,:),0]'); %back right
H_foot_fix(3,:,:) = Transl([foot.corners(3,:),0]'); %front left
H_foot_fix(4,:,:) = Transl([foot.corners(4,:),0]'); %front right

%desired angles
Qdes = Q_init;
Qstart = alg_to_rtbox(Qdes);

% cost function
w1 = 1000;
w2 = 0.01;
cost_fcn = @(z) (w1*sq_mat2(get_com_error(z(1:3),z(4:9),z(10:15),C_des,mlist))+w2*sq_mat2(Qdes-z(4:15)));

% nonlinear equality constraint
nlcstr_fcn = @(z)nlcstr_init(z(1:3),z(4:9),z(10:15),XLdes,XRdes,H_foot_fix);

% optimization options
options = optimoptions('fmincon','Algorithm','interior-point','Display','none');

% solve the optimization
tic
[zsol_ini,Jval_ini] = fmincon(cost_fcn,Xini,[],[],[],[],[],[],nlcstr_fcn,options);
toc
robot_model.DataFormat = 'col';
Qalg = zsol_ini(4:end);
Qrt = alg_to_rtbox(Qalg);
figure(4);
set(4,'DefaultFigureWindowStyle','docked');
show(robot_model,Qstart);
view([180 0]);
pause(1);
figure(4);
set(4,'DefaultFigureWindowStyle','docked');
show(robot_model,Qrt);
view([180 0]);
robot_model.DataFormat = 'struct';

%% setup the rest of the optimization
% allocate memory for the solution
for i=1:Nstates
    %the dimensionality is two times bigger as velocities are included
    state{i}.sol = zeros(2*Nz,state{i}.Npts);
end

%iterate through all the states
%init
qp_prev = zeros(size(Qdes));
for i = 1:Nstates
    curr_st = state{i};
    curr_ph = curr_st.phase;
    %shift weight only matters in SS phase
    if curr_st.shift_weight<0
        curr_sw = 'r';
    elseif curr_st.shift_weight>0
        curr_sw = 'l';
    else
        curr_sw = 'm';
    end
    
    Ni_pts = curr_st.Npts;
    for j = 1:Ni_pts
        %% nonlinear inequality constraint tasks
        if curr_ph=='d'
            %both feet are in full contact with the ground
            Fk_task_constr = [
                %left task
                curr_st.footstep.Ltraj.x(1,j);
                curr_st.footstep.Ltraj.x(2,j);
                zeros(4,1);
                %right task
                curr_st.footstep.Rtraj.x(1,j);
                curr_st.footstep.Rtraj.x(2,j);
                zeros(4,1);
            ];
        else
            if curr_sw=='l'
                %left foot is the support foot
                Fk_task_constr = [
                    %left task
                    curr_st.footstep.Ltraj.x(1:2,j);
                    zeros(4,1);
                    %right task is not active (set to zero on both sides)
                    zeros(6,1);
                ];                
            elseif curr_sw=='r'
                %right foot is the support foot
                Fk_task_constr = [
                    %left task is not active (set to zero on both sides)
                    zeros(6,1);
                    %right task
                    curr_st.footstep.Rtraj.x(1:2,j);
                    zeros(4,1);
                ];  
            end
        end
        
        nlcstr_fcn = @(z)nlcstr_mot_pl(z, curr_ph, curr_sw, Fk_task_constr);
        
        %% cost function tasks
        if curr_ph=='s'
            % only for single support we have tasks in a cost function
            if curr_sw=='l'
                % right leg is swinging
                % forward kinematics (position profile)
                Fk_task_cf = [
                    curr_st.footstep.Rtraj.x(1:2,j);            %x,y position
                    ones(4,1)*curr_st.footstep.Rtraj.x(3,j);    %z positions of 4 contact points
                ];
                % forward differential kinematics (velocity profile)
                FDk_task_cf = [
                    curr_st.footstep.Rtraj.xp(1:3,j);           %tra. velocity
                    zeros(3,1);                                 %rot. velocity        
                ];
            elseif curr_sw=='r'
                % left leg is swinging
                % forward kinematics (position profile)
                Fk_task_cf = [
                    curr_st.footstep.Ltraj.x(1:2,j);            %x,y position
                    ones(4,1)*curr_st.footstep.Ltraj.x(3,j);    %z positions of 4 contact points
                ];
                % forward differential kinematics (velocity profile)
                FDk_task_cf = [
                    curr_st.footstep.Ltraj.xp(1:3,j);           %tra. velocity
                    zeros(3,1);                                 %rot. velocity 
                ];
            end
        end
        
        curr_C_des = 0;
        curr_Cp_des = 0;
        
        %generate the cost function
        cost_fcn = @(z)costfun_mot_pl(z, curr_ph, curr_sw, curr_C_des, curr_Cp_des, Fk_task_cf, FDk_task_cf, qp_prev, mlist);
        
        %% solve the problem
        [zsol,Jval] = fmincon(cost_fcn,Xini,[],[],[],[],[],[],nlcstr_fcn,options);
        %extract previous velocities
        qp_prev = zsol(end-Nq,end);
    end
end