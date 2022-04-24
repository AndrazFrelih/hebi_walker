%% Load robot
roby = importrobot("florenceMatlab.urdf");
roby.DataFormat = 'column';
roby.Gravity = [0,0,-9.81];
%% Load trajectory
TG = load("TrGenGainTuning.mat");
TG = TG.TrajectoryGenerator;
%% Choose the desired state
st_ind = 3; %single support
st_des = TG.state_vector{st_ind};
st_qL = st_des.joint.q.L; %weight is on left side
st_qR = st_des.joint.q.R;

%% Allowed error in degrees
ang_deg = 0.5;
ang_rad = ang_deg*pi/180;

%% COM wrench
Wcom_SS = [
    0;0;300;0;0;0;
];

Npts_st = size(st_qL,2);
Kq = zeros(size(st_qL));
Dq = zeros(size(st_qL));

%% Stance foot tuning
for i=1:Npts_st
    Cfg = [st_qL(:,i);st_qR(:,i)];
    [Jac,Wext,Wlleg] = get_kine(roby,Cfg,Wcom_SS);
    Tor = Jac'*Wlleg;
    Kq(:,i) = abs(Tor/ang_rad);
    Dq(:,i) = 2*sqrt(Kq(:,i));
end

Kqmax_SF = max(Kq,[],2)/10;
Kqmax_SF(1) = Kqmax_SF(1)*30000;
Kqmax_SF(2) = Kqmax_SF(2)*200;
Kqmax_SF(3) = Kqmax_SF(3)*10;
Kqmax_SF(4) = Kqmax_SF(4)*20;
Kqmax_SF(5) = Kqmax_SF(5)*60;
Kqmax_SF(6) = Kqmax_SF(6)*30000;
Dqmax_SF = 2*sqrt(Kqmax_SF);
Dqmax_SF(2) = Dqmax_SF(2)*0.4;
Dqmax_SF(4) = Dqmax_SF(4)*0.1;
Dqmax_SF(5) = Dqmax_SF(5)*0.05;
Dqmax_SF(6) = Dqmax_SF(6)*0.2;

Kqmin_SF = min(Kq,[],2)/10;
Dqmin_SF = 2*sqrt(Kqmin_SF);


tspan = 0:0.002:2;

ind = 1;

Cfg = [st_qL(:,ind);st_qR(:,ind)];
Mod1 = @(t,y) model_gt(roby,t,y,Cfg,Kqmax_SF,Dqmax_SF, Wcom_SS);
yinit1 = [Cfg;zeros(2*size(st_qL,1),1)];
[ysol_rt1] = ode1(Mod1,tspan,yinit1);
ysol_rt1 = ysol_rt1';
vis_gt(roby,tspan,ysol_rt1);


%% Flight foot tuning
st_ind = 5; %single support
st_des = TG.state_vector{st_ind};
st_qL = st_des.joint.q.L; 
st_qR = st_des.joint.q.R; %weight is on right side
st_qpL = st_des.joint.qp.L; %weight is on left side
st_qpR = st_des.joint.qp.R;
qtraj = [st_qL;st_qR];
qptraj = [st_qpL;st_qpR];
tim = st_des.zmp.traj.t;

Kp = [20,70,50,30,30,1]'*3;
Kd = [1,5,10,10,5,1]'*1;

Mod2 = @(t,y) model_gt2(roby,t,y,qtraj,qptraj,Kp,Kd);
yinit2 = [qtraj(:,1);qptraj(:,1)];
[ysol_rt2] = ode1(Mod2,tim,yinit2);
ysol_rt2 = ysol_rt2';
vis_gt(roby,tim,ysol_rt2);
