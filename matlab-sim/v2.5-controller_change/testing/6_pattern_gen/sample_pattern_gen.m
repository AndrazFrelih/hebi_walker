Nstep = 9;
Nstep_ada = Nstep;

Zdes = 0.57; %desired COM height is set here (value is stored and then loaded in bipedal.m)
Tsample = 1/ctrl_config.fctrl;


%type of generated trajectory
app_type = 1;
%step characteristics
stepX = 0.2;
stepZ = 0.1;

if app_type==1
    %only shift weight
    Lx = 0; 
    Lz = 0;
elseif app_type == 2
    %step in place
    Lx = 0; 
    Lz = stepZ;
elseif app_type == 3
    %move 
    Lx = stepX; 
    Lz = stepZ;   
end
%Ly = (abs(Tf_ee_L(2,4))+abs(Tf_ee_R(2,4)));
Ly = 0.204;
dLy = Ly/2;

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

Tss = 0.1;
Tds = 1.5;
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
        
        if state{i}.nstep<Nstep_ada
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
    state{i}.zmp.traj.x(2,:) = state{i}.zmp.traj.x(2,:)*0.4;
end

%% calculate parameters for the preview controller
Tsim = state{Nstates}.Tstart + state{Nstates}.dT + Tfin;
Tpre = Tini; %preview time
Pctrl = calculate_preview(Zdes, Tsample, Tpre, Tsim);
Tsim_rob = Tsim - Tfin;

%% prepend the initial state (has to be added due to preview induced latency)
[state_ini] = gen_init_state(state{1},Tsample);
Nsamples = Nsamples + state_ini.Npts;
Nstates = Nstates + 1;
state = [{state_ini};state];


%% get full zmp trajectory
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

visualise = 0;
if visualise
    figure(1);
    plot(full_time,zmp_val);
    grid on
    figure(2);
    plot(full_time,lfoot_val);
    grid on
    figure(3);
    plot(full_time,rfoot_val);
    grid on
end

%simulation for x-com (since preview induces delay, start with the Npts-th
%delayed data point - this should allign the data
sim_input = timeseries(zmp_val(1,state_ini.Npts:end)',full_time(state_ini.Npts:end)'-Tini);
sim_X = sim('com_trajectory_gen','CaptureErrors','on');

%simulation for y-com (since preview induces delay, start with the Npts-th
%delayed data point - this should allign the data
sim_input = timeseries(zmp_val(2,state_ini.Npts:end)',full_time(state_ini.Npts:end)'-Tini);
sim_Y = sim('com_trajectory_gen','CaptureErrors','on');

%group the results into the structure as well
iend = 0;
for i=1:Nstates
    ist = iend+1;
    iend = iend + state{i}.Npts;
    state{i}.com.x = [
        sim_X.C.Data(ist:iend)';
        sim_Y.C.Data(ist:iend)';
        ones(1,state{i}.Npts)*Zdes;
    ];
    state{i}.com.xp = [
        sim_X.Cp.Data(ist:iend)';
        sim_Y.Cp.Data(ist:iend)';
        zeros(1,state{i}.Npts);
    ];
    state{i}.com.xpp = [
        sim_X.Cpp.Data(ist:iend)';
        sim_Y.Cpp.Data(ist:iend)';
        zeros(1,state{i}.Npts);
    ];
    state{i}.com.time = state{i}.zmp.traj.t;
end
%% calculate the initial pose of the robot:
% first a statically stable initial state is computed
C_des = [
    sim_X.C.Data(1);
    sim_Y.C.Data(1);
    Zdes;
];

% left foot task
XLdes = [
    lfoot_val(1,1); %initial X-position for center of foot
    lfoot_val(2,1); %initial Y-position for center of foot
    zeros(4,1);     %initial Z-position for all vertices
];

% right foot task
XRdes = [
    rfoot_val(1,1); %initial X-position for center of foot
    rfoot_val(2,1); %initial Y-position for center of foot
    zeros(4,1);     %initial Z-position for all vertices
];

% initial search value for the base position and joint angles
Xbase = [0;0;ctrl_config.base_height];
Xini = [Xbase;Q_init];
Nz = size(Xini,1); %dimensionality of the initial problem
Nq = size(Q_init,1); %dimensionality of the joint space


%desired angles
Qdes = Q_init;
Qstart = alg_to_rtbox(Qdes);

% cost function
w1 = 1000;
w2 = 0.01;
cost_fcn = @(z) (w1*sq_mat2(get_com_error(z(1:3),z(4:9),z(10:15),C_des,mlist))+w2*sq_mat2(Qdes-z(4:15)));

% nonlinear equality constraint
nlcstr_fcn = @(z)nlcstr_init(z(1:3),z(4:9),z(10:15),XLdes,XRdes);

% optimization options
%options = optimoptions('fmincon','Algorithm','interior-point','Display','none');
options = optimoptions('fmincon','Algorithm','sqp','Display','none');

% solve the optimization
%tic
[zsol_ini,Jval_ini] = fmincon(cost_fcn,Xini,[],[],[],[],[],[],nlcstr_fcn,options);
%toc

%visualise the result
robot_model.DataFormat = 'col';
Qalg = zsol_ini(4:end);
Qrt = alg_to_rtbox(Qalg);
figure(4);
set(4,'DefaultFigureWindowStyle','docked');
show(robot_model,Qstart);
view([180 0]);
pause(0.1);
figure(4);
set(4,'DefaultFigureWindowStyle','docked');
show(robot_model,Qrt);
view([180 0]);
robot_model.DataFormat = 'struct';

%% full body trajectory generation
%initialize the sequential optimization
zini = [
    zsol_ini;               %positions
    zeros(size(zsol_ini));  %velocities
];  

static_pg = 1; % use the static (1) or kinematic (0) case
regen_data = 1; % generate data anew?

if static_pg
    %generate static poses and discretely derive the velocities
    if regen_data
        %% generate data again
        disp("Trajectory generation took:");
        tic
        sol_arr = opt_stat(robot_model,state,zsol_ini,mlist,options,Tsample);
        toc
        visualise_solution(robot_model,sol_arr,state,1,Tsample,90);
    else
        %% load old data
        data = load("Static_pattern_generation.mat");
        sol_arr = data.sol_arr;
        state = data.state;
        robot_model = data.robot_model;
        Nstates = size(state,1);
    end
    
    [sol_arr_ext] = compute_vels(sol_arr,state,Nstates,Nz,Tsample);
    
    for i=1:Nstates
        %write fb translation solutions to the state vector
        state{i}.fb.x = sol_arr_ext{i}(1:3,:);
        state{i}.fb.xp = sol_arr_ext{i}(Nz+1:Nz+3,:);
        %write angle solutions to the state vector
        q = sol_arr_ext{i}(4:Nz,:);
        state{i}.joint.q.L = q(1:end/2,:);
        state{i}.joint.q.R = q(end/2+1:end,:);
        qp = sol_arr_ext{i}(Nz+4:end,:);
        state{i}.joint.qp.L = qp(1:end/2,:);
        state{i}.joint.qp.R = qp(end/2+1:end,:);
    end
    
    %% generate timeseries for bipedal_robot.slx
    [BipedGeneratedTrajectory] = gen_timeseries(state,full_time);
    TrajectoryGenerator.timeseries = BipedGeneratedTrajectory;
    TrajectoryGenerator.state_vector = state;
    TrajectoryGenerator.test_type = app_type;
    TrajectoryGenerator.Tsim = Tsim_rob;
    save('./saved_values/TrGen.mat','TrajectoryGenerator');
else
    %kinematically consistent PG - not working properly
    sol_arr = opt_velo(robot_model,state,zini,mlist,options, Tsample);
    visualise_solution(robot_model,sol_arr,state,0,90);
end
