% load the pregenerated trajectory
if ~exist('TG','var')
    %load the trajectory
    TG = load("TrGen.mat");
    TG = TG.TrajectoryGenerator;
    %lead the masses
    ML = load("masslist.mat");
    ML = ML.mlist;
end
traj = TG.timeseries;

% number of joints
Nq = 12;

% number of optimisation steps
Nop_st = max(size(traj.Data(:,1)));

% load some parameters
fs = 500;
dt = 1/fs;

%% tasks can be pregenerated here as there is no feedback
% indices
iZmp = 4:5;
iCom = 6:7;
iComP = 8:9;
iPl = 34:36;
iPr = 37:39;
iPlP = 40:42;
iPrP = 43:45;

% get desired values for tasks
Zmp_des = traj.Data(:,iZmp);
Com_des = traj.Data(:,iCom);
ComP_des = traj.Data(:,iComP);
Pl_des = traj.Data(:,iPl);
Pr_des = traj.Data(:,iPr);
PlP_des = traj.Data(:,iPlP);
PrP_des = traj.Data(:,iPrP);

% generate tasks
TaskCom = [ComP_des';zeros(1,Nop_st)];
TaskEEL = [
    PlP_des';
    zeros(size(PlP_des))'; %no rotation of feet is desired
];
TaskEER = [
    PrP_des';
    zeros(size(PrP_des))'; %no rotation of feet is desired
];
TaskOri = zeros(size(TaskCom));

TaskJoint = zeros(Nq,1); %this is implicitly a position control part

%% setup optimization
% starting value for configuration
q0 = [
    TG.state_vector{1}.fb.x(:,1);
    zeros(3,1);
    TG.state_vector{1}.joint.q.L(:,1);
    TG.state_vector{1}.joint.q.R(:,1);
];
qfull = zeros(size(q0,1),Nop_st+1);

% starting point of optimization
w0 = zeros(size(q0));
wfull = zeros(size(q0,1),Nop_st);


% create optimisation object
osqp_obj = osqp;
osqp_set = osqp_obj.default_settings();
osqp_set.verbose = 0; %do not print output
osqp_set.eps_rel = 1e-12;
osqp_set.eps_abs = 1e-12;
osqp_set.adaptive_rho = 0;
osqp_set.polish = 1;

%% start the process
tic
for i = 1:Nop_st
    %% initial values for optimization
    if i==1
        q_it = q0;
        w_it = w0;
    else
        q_it = q_it + w_sol.x*dt;
        w_it = w_sol.x;
    end
    qfull(:,i) = q_it;
    wfull(:,i) = w_it;
    
    %% compute necessary jacobians
    [Jw_com, Jw_eeL, Jw_eeR, Jw_b] = get_jacobians(q_it,ML);
    
    %% setup of Px, qx and Ax matrices and bounds
    % Px
    R1x = Jw_b(4:6,:);
    W1x = eye(min(size(R1x)))*1000;
    s1x = TaskOri(:,i);
    P1x = R1x'*W1x*R1x;
    q1x = R1x'*W1x*s1x;

    R2x = [
        zeros(Nq,6), eye(Nq);
    ];
    W2x = eye(min(size(R2x)))/100; %do not include the task as here measurements are not available
    s2x = TaskJoint;
    P2x = R2x'*W2x*R2x;
    q2x = R2x'*W2x*s2x;

    %R3x = Jw_com(1:3,:);
    %W3x = eye(min(size(R3x)))*100;
    %s3x = TaskCom(:,i);
    %P3x = R3x'*W3x*R3x;
    %q3x = R3x'*W3x*s3x;

    Px = P1x + P2x; %+ P3x;
    qx = q1x + q2x; %+ q3x;

    % Ax formulation
    A1x = Jw_com(1:3,:);
    l1x = TaskCom(:,i);
    h1x = l1x;

    A2x = Jw_eeL;
    l2x = TaskEEL(:,i);
    h2x = l2x;

    A3x = Jw_eeR;
    l3x = TaskEER(:,i);
    h3x = l3x;
    
    qpmax = 100;
    A4x = R2x;
    l4x = -qpmax*ones(Nq,1);
    h4x = +qpmax*ones(Nq,1);

    % combine them
    Ax = [A1x;A2x;A3x;A4x];
    lx = [l1x;l2x;l3x;l4x];
    hx = [h1x;h2x;h3x;h4x];

    %% update the optimisation object
    if i==1
        osqp_obj.setup(Px,qx,Ax,lx,hx,osqp_set);
    else
        osqp_obj.update('Px',Px,'q',qx,'Ax',Ax,'l',lx,'u',hx);
    end
    
    % solve the qp problem
    w_sol = osqp_obj.solve();
    if isnan(w_sol.x)
        disp("Error in optimisation!");
        break;
    end
end
toc
