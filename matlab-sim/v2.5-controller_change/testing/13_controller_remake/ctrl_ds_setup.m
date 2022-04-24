%% General settings and parameters
Zdes = 0.56;

ctrl_cfg_rem.par.Ts = 1/fs;
ctrl_cfg_rem.par.Nq = 12;

%% Finite state machine
ctrl_cfg_rem.fsm.test_type = 1;

if ctrl_cfg_rem.fsm.test_type == -1
    ctrl_cfg_rem.fsm.use_ref = 1;
    ctrl_cfg_rem.fsm.Tpause = 0.0; %initial pause for the fb estimation
else
    ctrl_cfg_rem.fsm.use_ref = 0;
    ctrl_cfg_rem.fsm.Tpause = 0.5; %initial pause for the fb estimation
end

[FSM] = fsm_ctrl_struct;
FSM_info = Simulink.Bus.createObject(FSM);
FSM_bus = evalin('base', FSM_info.busName);

%% Filter
ctrl_cfg_rem.fil.Tfil = Tfil;

%% Pattern generation settings
Nsteps = 9;
Times.Tsample = ctrl_cfg_rem.par.Ts; 
Times.Tss = 0.5;
Times.Tds = 1.5;
Times.Tini = 0.5;
Times.Tfin = 1;
ctrl_cfg_rem.fsm.times = Times;


[state_list,Nsamples] = pregen_pattern(Nsteps, Times, ctrl_cfg_rem.fsm.test_type);
Tsim = Nsamples*1/fs + ctrl_cfg_rem.fsm.Tpause;

pg_traj =  create_traj(state_list, Nsamples);
pg_traj_info = Simulink.Bus.createObject(pg_traj);
pg_traj_bus = evalin('base', pg_traj_info.busName);

%create phase data struct
[ph_data_bus] = ph_data_bus_creator;
Nprev = floor(Times.Tini/Times.Tsample);
ctrl_cfg_rem.pg.ini_targs = repmat([pg_traj.foot.xL(:,1);pg_traj.foot.xR(:,1);pg_traj.foot.xLp(:,1);pg_traj.foot.xRp(:,1)],1,Nprev);

%% Trajectory generation settings
ctrl_cfg_rem.tg = calculate_preview2(Zdes, Times.Tsample, Times.Tini,Tsim,Nprev);

% output bus
[tg_out_bus] = tg_out_bus_creator;

%% Floating base estimation settings
ctrl_cfg_rem.fb_est.Ts = 1/ctrl_config.fctrl;
ctrl_cfg_rem.fb_est.foot_data = ctrl_config.foot_data;
ctrl_cfg_rem.fb_est.mlist = ctrl_config.mlist;
ctrl_cfg_rem.fb_est.omega = ctrl_cfg_rem.tg.par.omega;
ctrl_cfg_rem.fb_est.iir.alpha_cp = 0.02;
ctrl_cfg_rem.fb_est.iir.alpha_cpp = 0.001;

% foot drift compensation factor
ctrl_cfg_rem.fb_est.alpha = 0.005;

com = [0;0;Zdes];
xL = pg_traj.foot.xL(:,1);
xR = pg_traj.foot.xR(:,1);
xinit_opt = [0;0;ctrl_config.base_height];
qinit_opt = [QL_sym';QR_sym'];
[xfb_sol,qL_sol,qR_sol] = initial_config(xinit_opt,qinit_opt, com, xL, xR,ctrl_config.mlist);
INIT.x = xfb_sol;
INIT.L = qL_sol;
INIT.R = qR_sol;
save('./saved_values/INIT.mat','INIT');

% initial base 2 world tf
Hw_b_ini = Hbase_in([xfb_sol;zeros(3,1)]);
% initial ee 2 base tfs
HLlist=H_abs_bL(qL_sol);
HRlist=H_abs_bR(qR_sol);
Hb_eeL_ini = reshape(HLlist(end,:,:),4,4);
Hb_eeR_ini = reshape(HRlist(end,:,:),4,4);
% initial ee 2 world tfs
Hw_eeL_ini = Hw_b_ini*Hb_eeL_ini;
Hw_eeR_ini = Hw_b_ini*Hb_eeR_ini;

% initial values


ctrl_cfg_rem.fb_est.init.MSL = lleg_str;
ctrl_cfg_rem.fb_est.init.MSR = rleg_str;
ctrl_cfg_rem.fb_est.init.FSL = lleg_FS_str;
ctrl_cfg_rem.fb_est.init.FSR = rleg_FS_str;
ctrl_cfg_rem.fb_est.init.com = [0;0;Zdes];
ctrl_cfg_rem.fb_est.init.Hw_b = Hw_b_ini;
ctrl_cfg_rem.fb_est.init.Hw_eeL = Hw_eeL_ini;
ctrl_cfg_rem.fb_est.init.Hw_eeR = Hw_eeR_ini;

ctrl_cfg_rem.fb_est.cdf_data = cdf_data;

% create a bus for jacobians
[jac_bus] = jac_bus_creator;

% create bus for measurements
[meas_bus] = meas_bus_creator;

% bus for contact data
[cd_bus] = cd_bus_creator;

%% Balancer
%ctrl_cfg_rem.bal.com_height = ctrl_cfg_rem.tg.par.zc;
ctrl_cfg_rem.bal.com_height = Zdes;
ctrl_cfg_rem.bal.mlist = ctrl_cfg_rem.fb_est.mlist;
ctrl_cfg_rem.bal.foot_data = ctrl_cfg_rem.fb_est.foot_data;


%% Inverse kinematics settings
ctrl_cfg_rem.ik_opt.q_ini = [xfb_sol;zeros(3,1);qL_sol;qR_sol];
ctrl_cfg_rem.ik_opt.qp_ini = zeros(size(ctrl_cfg_rem.ik_opt.q_ini));
ctrl_cfg_rem.ik_opt.qp_lim = 5;

% create a bus for input data
[ikine_inp_bus] = ikine_inp_bus_creator;

%% Static force compensation settings
ctrl_cfg_rem.sf_comp.gs = ctrl_config.damper; %gas spring model
ctrl_cfg_rem.sf_comp.bc = 0; %bungee cord model

% controller 
ctrl_cfg_rem_info = Simulink.Bus.createObject(ctrl_cfg_rem);
ctrl_cfg_rem_bus = evalin('base', ctrl_cfg_rem_info.busName);


%% Load the robot model for the gravity compensation
roby = importrobot("florenceMatlab.urdf");
roby.DataFormat = 'column';
roby.Gravity = [0,0,-g];



%% Setup variants
ctrl_variants;
