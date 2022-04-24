%% VERSION 2.5.

addpath(genpath('cad_models'));
addpath(genpath('testing'));
addpath(genpath('functions'));
addpath(genpath('symbolic_functions'));
addpath(genpath('urdf'));
addpath(genpath('saved_values'));
addpath(genpath('c_source'));
addpath(genpath('/home/isclab/Documents/andraz-f/osqp'));
sim_settings;

%% SOME SIMULATION SETTINGS
use_reference_FBdata = 0; %if 1, true Hwb is used, if 0, reconstructed one is used

%% SAMPLING
fs = 1000;
Ts = 1/fs;


%% UNITS
COMunit = 1e-3;
Munit = 1e-3;
Iunit = 1e-9;

%% ENV. & ROB. DIMS
dimensions;
sim_floor.dims = [2,2,0.1];

%% MOTOR DATA
motor.X8.offs = 0.038;
motor.X8.v16.mass = 0.455;
motor.X8.v16.Tmax = 200;

%% TRAJECTORY GENERATOR DATA
INIT_vals = load("INIT.mat");
INIT_vals = INIT_vals.INIT;

%% JOINT ANGLES
%Home is robot's home configuration. Init can be any configuration, with
%which the simulation should be started. In this case, robot's home and
%init configs are the same...
% left side
QL_sym = [3,0,-45,80,-35,0]*pi/180;
QL_home = INIT_vals.L;
QL_init = QL_home;


% right side
QR_sym = [-3,0,-45,80,35,0]*pi/180;
QR_home = INIT_vals.R;
QR_init = QR_home;

% full
Q_init = [QL_init;QR_init];

%% BASE
Base.Xinit = INIT_vals.x;
Base.dims = [0.2,0.2,0.1]; %contacts
Base.body.com = [6.638, -0.196, 54.541]*COMunit;
Base.body.mass = 1.837E+04 * Munit;
Base.body.inertia = [1.768E+08,1.913E+08,2.864E+08,-1.043E+06,2.306E+07,1.130E+04] * Iunit;
Base.body_rt = Base.body;

% link.body is the first body attached to the follower side of the joint

%% JOINT 1 -> HIP YAW >>>>> CHECKED
% left leg
LefHipYaw.pos = [+dim.L1,+dim.L2,-dim.L3];
LefHipYaw.rot = [pi,0,0];
LefHipYaw.body.com = [-48.962 , 10.242 , 91.397]*COMunit;
LefHipYaw.body.mass = 760.027 * Munit;
LefHipYaw.body.inertia = [9.643E+05, 8.531E+05, 8.291E+05, 9.659E+04, 2.206E+05, -1.310E+04] * Iunit;
LefHipYaw.body_rt = LefHipYaw.body;
LefHipYaw.IMU.pos = zeros(1,3);
LefHipYaw.IMU.rot = [0,0,-pi/6];
LefHipYaw.qinit = QL_init(1);

% right leg
RigHipYaw.pos = [+dim.L1,-dim.L2,-dim.L3];
RigHipYaw.rot = [pi,0,0];
RigHipYaw.body.com = [-48.962 , 10.242 , 91.397]*COMunit;
RigHipYaw.body.mass = 760.027 * Munit;
RigHipYaw.body.inertia = [9.643E+05, 8.531E+05, 8.291E+05, -5.424E+04, 2.261E+04, 8.598E+04] * Iunit;
RigHipYaw.body_rt = RigHipYaw.body;
RigHipYaw.IMU.pos = zeros(1,3);
RigHipYaw.IMU.rot = [0,0,pi/6];
RigHipYaw.qinit = QR_init(1);

%% JOINT 2 -> HIP ROLL (SAME LINK FOR L & R)
% left leg
LefHipRoll.pos = [-dim.L6,0,dim.L4];
LefHipRoll.rot = [0,-pi/2,0];
LefHipRoll.body.com = [10.985, -0.203, 95.13] * COMunit;
LefHipRoll.body.mass = 1427.87 * Munit;
LefHipRoll.body.inertia = [5.117E+06, 1.731E+06, 5.200E+06, 1000.497, -6.111E+04, 1.050E+05] * Iunit;
LefHipRoll.body_rt = LefHipRoll.body;
LefHipRoll.IMU.pos = zeros(1,3);
LefHipRoll.IMU.rot = [0,0,-pi/2];
LefHipRoll.qinit = QL_init(2);

% right leg
RigHipRoll.pos = [-dim.L6,0,dim.L4];
RigHipRoll.rot = [0,-pi/2,0];
RigHipRoll.body = LefHipRoll.body;
RigHipRoll.body_rt = RigHipRoll.body;
RigHipRoll.IMU.pos = zeros(1,3);
RigHipRoll.IMU.rot = [0,0,+pi/2];
RigHipRoll.qinit = QR_init(2);

%% JOINT 3 -> HIP PITCH - two motors per leg (SAME LINK FOR L & R) >>>>>CHECKED
% left leg
LefHipPitch.pos = [0,0,dim.L7];
LefHipPitch.rot = [-pi/2,0,0];
LefHipPitch.pos_1 = [0,0,dim.L5];
LefHipPitch.rot_1 = [0,0,0];
LefHipPitch.pos_2 = [0,0,-dim.L5];
LefHipPitch.rot_2 = [pi,0,0];
LefHipPitch.body.com = [257.774 , 1.006 , 36.166] * COMunit;
LefHipPitch.body.mass = 882.958 * Munit;
LefHipPitch.body.inertia = [6.454E+05, 1.056E+07, 1.046E+07, -1.392E+04, 9.962E+05, 9.962E+05] * Iunit;
LefHipPitch.body_rt.com = [257.774, 0.00, 0.00] * COMunit;
LefHipPitch.body_rt.mass = LefHipPitch.body.mass*2; %full mass of both parts
LefHipPitch.body_rt.inertia = [1.088E+07, 3.071E+07, 2.091E+07, 0, 0, -1.092E+05] * Iunit; %full inertia of both parts (POI compared to MOI are so small, that they are set to zero)
LefHipPitch.IMU_1.pos = zeros(1,3);
LefHipPitch.IMU_1.rot = [0,0,pi];
LefHipPitch.IMU_2.pos = zeros(1,3);
LefHipPitch.IMU_2.rot = [0,0,pi];
LefHipPitch.qinit_1 = QL_init(3); %motor L3_1
LefHipPitch.qinit_2 = -LefHipPitch.qinit_1; %motor L3_2
LefHipPitch.qinit = LefHipPitch.qinit_1;
% right leg
RigHipPitch.pos = LefHipPitch.pos;
RigHipPitch.rot = LefHipPitch.rot;
RigHipPitch.pos_1 = LefHipPitch.pos_1;
RigHipPitch.rot_1 = LefHipPitch.rot_1;
RigHipPitch.pos_2 = LefHipPitch.pos_2;
RigHipPitch.rot_2 = LefHipPitch.rot_2;
RigHipPitch.body = LefHipPitch.body; %same as left
RigHipPitch.body_rt = LefHipPitch.body_rt; %same as left
RigHipPitch.IMU_1.pos = zeros(1,3);
RigHipPitch.IMU_1.rot = [0,0,pi];
RigHipPitch.IMU_2.pos = zeros(1,3);
RigHipPitch.IMU_2.rot = [0,0,pi];
RigHipPitch.qinit_1 = QR_init(3); %motor R3_1
RigHipPitch.qinit_2 = -RigHipPitch.qinit_1; %motor R3_2
RigHipPitch.qinit = RigHipPitch.qinit_1;

%% JOINT 4 -> KNEE PITCH - two motors >>>>>CHECKED
% left leg
LefKnee.pos_virt = [dim.L8,0,0];
LefKnee.pos = [dim.L8,0,dim.L9-dim.L5];
LefKnee.rot = [pi,0,0];
LefKnee.pos_1 = [0,0,dim.L9];
LefKnee.rot_1 = [pi,0,0];
LefKnee.pos_2 = [0,0,dim.L9];
LefKnee.rot_2 = [0,0,0];
LefKnee.body.com = [199.549 , 0.615 , 65.412] * COMunit;
LefKnee.body.mass = 1472.442 * Munit;
LefKnee.body.inertia = [1.379E+06, 2.557E+07, 2.495E+07, -6.728E+04, 2.696E+06, 3.448E+04] * Iunit;
LefKnee.body_rt.com = [199.549, -0.615, 17.138] * COMunit;
LefKnee.body_rt.mass = LefKnee.body.mass;
LefKnee.body_rt.inertia = LefKnee.body.inertia; %both are expressed in COM cf
LefKnee.IMU_1.pos = zeros(1,3);
LefKnee.IMU_1.rot = [0,0,pi/2];
LefKnee.IMU_2.pos = zeros(1,3);
LefKnee.IMU_2.rot = [0,0,-pi/2];
LefKnee.qinit_1 = -QL_init(4); %motor L4_1
LefKnee.qinit_2 = -LefKnee.qinit_1; %motor L4_2
LefKnee.qinit = LefKnee.qinit_2;


% right leg
RigKnee.pos_virt = LefKnee.pos_virt;
RigKnee.pos = LefKnee.pos;
RigKnee.rot = LefKnee.rot;
RigKnee.pos_1 = LefKnee.pos_1;
RigKnee.rot_1 = LefKnee.rot_1;
RigKnee.pos_2 = LefKnee.pos_2;
RigKnee.rot_2 = LefKnee.rot_2;
RigKnee.body = LefKnee.body;
RigKnee.body_rt.com = [199.549, 0.615, -17.138]* COMunit;
RigKnee.body_rt.mass = LefKnee.body_rt.mass;
RigKnee.body_rt.inertia = LefKnee.body_rt.inertia;
RigKnee.IMU_1.pos = zeros(1,3);
RigKnee.IMU_1.rot = [0,0,pi/2];
RigKnee.IMU_2.pos = zeros(1,3);
RigKnee.IMU_2.rot = [0,0,-pi/2];
RigKnee.qinit_1 = -QR_init(4); %motor R4_1/30
RigKnee.qinit_2 = -RigKnee.qinit_1; %motor R4_2
RigKnee.qinit = RigKnee.qinit_2;

%% JOINT 5 -> ANKLE PITCH >>>>>CHECKED
% left leg
LefAnklePitch.pos = [dim.L10,0,+dim.L11];
LefAnklePitch.rot = [0,0,0];
LefAnklePitch.body.com = [-10.242, 5.25, 91.397] * COMunit;
LefAnklePitch.body.mass = 760.027 * Munit;
LefAnklePitch.body.inertia = [6.608E+05, 9.643E+05, 6.368E+05, -5.153E+04, 1.311E+04, 1.821E+04] * Iunit;
LefAnklePitch.body_rt = LefAnklePitch.body;
LefAnklePitch.IMU.pos = zeros(1,3);
LefAnklePitch.IMU.rot = zeros(1,3);
LefAnklePitch.qinit = QL_init(5);
% right leg
RigAnklePitch.pos = [dim.L10,0,-dim.L11];
RigAnklePitch.rot = [pi,0,0];
RigAnklePitch.body.com = [-10.242 , -5.632 , 93.734] * COMunit;
RigAnklePitch.body.mass = 760.027 * Munit;
RigAnklePitch.body.inertia = [6.854E+05, 9.909E+05, 6.348E+05, 5.028E+04, 8.723E+04, -2.275E+04] * Iunit;
RigAnklePitch.body_rt = RigAnklePitch.body;
RigAnklePitch.IMU.pos = zeros(1,3);
RigAnklePitch.IMU.rot = zeros(1,3);
RigAnklePitch.qinit = QR_init(5);

%% JOINT 6 -> ANKLE ROLL >>>>>CHECKED
% left leg
LefAnkleRoll.pos = [0,+dim.L13,dim.L12];
LefAnkleRoll.rot = [+pi/2,0,0];
LefAnkleRoll.body.com = [88.122 , -0.14 , 34.447] * COMunit;
LefAnkleRoll.body.mass = 1388.81 * Munit;
LefAnkleRoll.body.inertia = [4.344E+06, 3.525E+06, 3.313E+06, 1012.806, 2.928E+05, 9093.37] * Iunit;
LefAnkleRoll.body_rt = LefAnkleRoll.body;
LefAnkleRoll.IMU.pos = zeros(1,3);
LefAnkleRoll.IMU.rot = zeros(1,3);
LefAnkleRoll.qinit = QL_init(6);
% right leg
RigAnkleRoll.pos = [0,-dim.L13,dim.L12];
RigAnkleRoll.rot = [-pi/2,0,0];
RigAnkleRoll.body = LefAnkleRoll.body;
RigAnkleRoll.body_rt = RigAnkleRoll.body;
RigAnkleRoll.IMU.pos = zeros(1,3);
RigAnkleRoll.IMU.rot = zeros(1,3);
RigAnkleRoll.qinit = QR_init(6);

%% END EFFECTOR POSITION
LefFoot.pos = [dim.L15,0,dim.L14];
LefFoot.rot = [0,-pi/2,pi];
RigFoot.pos = LefFoot.pos;
RigFoot.rot = LefFoot.rot;

%% GAS SPRING-DAMPER DEFINITION
offs1 = 17.269e-3;
offs2 = 11.269e-3;
% hip connection
% left
LefDamper.hip.pos = [-25e-3,+30e-3,+140.05e-3];
LefDamper.hip.rot = [pi/2,0,0];
LefDamper.hip.offs = offs1;
%screw
LefDamper.hip.body1.com = [0, 0, -9.648]*COMunit;
LefDamper.hip.body1.mass = 13.304*Munit;
LefDamper.hip.body1.inertia = [786.345,789.393,193.444,0,0,0]*Iunit;
%sleeve
LefDamper.hip.body2.com = [115.546, 0.034, -0.011]*COMunit;
LefDamper.hip.body2.mass = 401.692*Munit;
LefDamper.hip.body2.inertia = [1.895E+04,1.660E+06,1.660E+06,0,1051.977,0]*Iunit;
%right
RigDamper.hip.pos = [-25e-3,-30e-3,+140.05e-3];
RigDamper.hip.rot = [-pi/2,0,0];
RigDamper.hip.offs = offs1;
%screw
RigDamper.hip.body1 = LefDamper.hip.body1;
%sleeve
RigDamper.hip.body2 = LefDamper.hip.body2;

% translational link transformation
%spring model
gas_DS_model.L0 = 130e-3;
gas_DS_model.F0 = 90;
gas_DS_model.D = 0;
%left
LefDamper.tra.pos = [248.158e-3,0,0];
LefDamper.tra.rotU = [pi,pi/2,0];
LefDamper.tra.rotL = [0,pi/2,0];
LefDamper.tra.lim.max = 130e-3;
LefDamper.tra.lim.min = 0;
LefDamper.tra.mod = gas_DS_model;
%right
RigDamper.tra = LefDamper.tra;

% knee connection
%left
%LefDamper.knee.pos = [100e-3,+10e-3,-27.409e-3]; conc. model
LefDamper.knee.pos = [100e-3,+10e-3,55.141e-3];
LefDamper.knee.rot = [0,pi,0];
LefDamper.knee.offs = offs2;
%screw
LefDamper.knee.body1.com = [0.065, -0.217, -7.054]*COMunit;
LefDamper.knee.body1.mass = 8.52*Munit;
LefDamper.knee.body1.inertia = [550.141,553.189,68.662,0,0,0]*Iunit;
%insert
LefDamper.knee.body2.com = [-77.224, 0, -0.11]*COMunit;
LefDamper.knee.body2.mass = 73.152*Munit;
LefDamper.knee.body2.inertia = [735.32,2.069E+05,2.069E+05,0,0,-703.08]*Iunit;

%right
%RigDamper.knee.pos = [100e-3,-10e-3,-27.409e-3]; conc. model
RigDamper.knee.pos = [100e-3,-10e-3,55.141e-3];
RigDamper.knee.rot = [0,pi,0];
RigDamper.knee.offs = offs2;
%screw
RigDamper.knee.body1 = LefDamper.knee.body1;
%insert
RigDamper.knee.body2 = LefDamper.knee.body2;


%% FULL MASS
mlist = [
    Base.body.mass;
    LefHipYaw.body.mass;
    LefHipRoll.body.mass;
    LefHipPitch.body.mass;
    LefKnee.body.mass;
    LefAnklePitch.body.mass;
    LefAnkleRoll.body.mass;
    RigHipYaw.body.mass;
    RigHipRoll.body.mass;
    RigHipPitch.body.mass;
    RigKnee.body.mass;
    RigAnklePitch.body.mass;
    RigAnkleRoll.body.mass;
];
save("./saved_values/masslist.mat","mlist");
ll_mass = sum(mlist(2:7));
rl_mass = sum(mlist(8:end));
total_mass = mlist(1) + ll_mass + rl_mass;

%% contact penalty definition
foot.dims = [0.158,0.14,0];
foot.corners = [
    -foot.dims(1)/2, -foot.dims(2)/2;
    -foot.dims(1)/2, +foot.dims(2)/2;
    +foot.dims(1)/2, -foot.dims(2)/2;
    +foot.dims(1)/2, +foot.dims(2)/2;
];

foot.edge_rad = 0.033;
foot.cont_rad = 0.025/2;
%calculate the position of the contact points
Pedge = foot.dims(1:2);
foot.dims_ada = [Pedge-[1,1]*((1-1/sqrt(2))*foot.edge_rad+foot.cont_rad),0];
foot.dims_ada = foot.dims;
pen_steady_state = 0.005;
%other constants
g = 9.80665;
gvec = [0 0 -g];
Kcont = total_mass*g/pen_steady_state;
Dcont = Kcont/10;
Tfil = Ts*10;

%force sensor position
fsen_X = 0.052;
fsen_Y = 0.046;
Areac_meas = [
    +fsen_X,+fsen_Y,1;
    +fsen_X,-fsen_Y,1;
    -fsen_X,-fsen_Y,1;
    -fsen_X,+fsen_Y,1;
];

%measurement points
foot.fsen.Pmeas = Areac_meas(:,1:2);

AreacX_meas = [
    +fsen_X,1;
    -fsen_X,1
];

AreacY_meas = [
    +fsen_Y,1;
    -fsen_Y,1
];

%force remapping from measurement to edge position
foot.zmp.Am.x = AreacX_meas;
foot.zmp.Am.y = AreacY_meas;
foot.zmp.Am.full = Areac_meas;
foot.zmp.Am.xinv = inv(AreacX_meas);
foot.zmp.Am.yinv = inv(AreacY_meas);
foot.zmp.Am.finv = (Areac_meas'*Areac_meas)\Areac_meas';

%the reaction force is located in the four points at foot edges
Areac_edge = [
    +foot.dims_ada(1)/2,+foot.dims_ada(2)/2,1;
    +foot.dims_ada(1)/2,-foot.dims_ada(2)/2,1;
    -foot.dims_ada(1)/2,-foot.dims_ada(2)/2,1;
    -foot.dims_ada(1)/2,+foot.dims_ada(2)/2,1
];

%edges of the foot
foot.fsen.Pedge = Areac_edge(:,1:2);

AreacX_edge = [
    +foot.dims_ada(1),1;
    -foot.dims_ada(1),1
];

AreacY_edge = [
    +foot.dims_ada(2),1;
    -foot.dims_ada(2),1
];

%force remapping from edge to measurement position
foot.zmp.Ae.x = AreacX_edge;
foot.zmp.Ae.y = AreacY_edge;
foot.zmp.Ae.full = Areac_edge;
foot.zmp.Ae.xinv = inv(AreacX_edge);
foot.zmp.Ae.yinv = inv(AreacY_edge);
foot.zmp.Ae.finv = (Areac_edge'*Areac_edge)\Areac_edge';

%% ROBOT MECHANIC INFORMATION:
robot.mech.base = Base;
robot.mech.left.q1_hip_yaw = LefHipYaw;
robot.mech.left.q2_hip_roll = LefHipRoll;
robot.mech.left.q3_hip_pitch = LefHipPitch;
robot.mech.left.q4_knee_pitch = LefKnee;
robot.mech.left.q5_ankle_pitch = LefAnklePitch;
robot.mech.left.q6_ankle_roll = LefAnkleRoll;
robot.mech.left.ee = LefFoot;
robot.mech.right.q1_hip_yaw = RigHipYaw;
robot.mech.right.q2_hip_roll = RigHipRoll;
robot.mech.right.q3_hip_pitch = RigHipPitch;
robot.mech.right.q4_knee_pitch = RigKnee;
robot.mech.right.q5_ankle_pitch = RigAnklePitch;
robot.mech.right.q6_ankle_roll = RigAnkleRoll;
robot.mech.right.ee = RigFoot;

robot.mech.foot = foot;

robot.damper.left = LefDamper;
robot.damper.right = RigDamper;

%% CREATE ROBOT MODEL:
[robot_model, robot_model_dyn, frame_list] = create_robot_model(robot.mech,1);
%home configuration
Q_config = homeConfiguration(robot_model);
Tf_ee_L = getTransform(robot_model,Q_config,frame_list.links.lleg(7),frame_list.links.base);
Tf_ee_R = getTransform(robot_model,Q_config,frame_list.links.rleg(7),frame_list.links.base);

% average projection of end effector translation vectors in world cf
pdes_ini = zeros(2,1);
zL = -Tf_ee_L(3,4);
zR = -Tf_ee_R(3,4);
%they should be the same:
if zL>zR
    zdes_base = zL;
elseif zR>zL
    zdes_base = zR;
else
    zdes_base = (zL+zR)/2;
end

%use the Rtoolbox model to get the COM
robot_model.DataFormat = 'col';
COMbase = centerOfMass(robot_model,alg_to_rtbox(Q_init));
robot_model.DataFormat = 'struct';

COMbase = compute_com_pos2([INIT_vals.x;zeros(3,1)],INIT_vals.L,INIT_vals.R,mlist,'b');
COMworld = compute_com_pos2([INIT_vals.x;zeros(3,1)],INIT_vals.L,INIT_vals.R,mlist,'w');
%create control configuration structure:
ctrl_config.base_height = COMworld(3)-COMbase(3);
%ctrl_config.com_height = zdes_base+COMbase(3); %adapt the value
ctrl_config.com_height = COMworld(3);

ctrl_config.mass = total_mass;
ctrl_config.mlist = mlist;
ctrl_config.foot_data = foot;
ctrl_config.pdes_ini = pdes_ini;
ctrl_config.fctrl = fs;

ctrl_config.qLd = QL_home;
ctrl_config.qRd = QR_home;

ctrl_config.damper.L0 = gas_DS_model.L0;
ctrl_config.damper.F0 = gas_DS_model.F0;


ctrl_cfg_info = Simulink.Bus.createObject(ctrl_config);
ctrl_cfg_bus = evalin("base", ctrl_cfg_info.busName);

%define initial values for required robotic structures
JLinit_ctrl = geometricJacobian(robot_model, Q_config, "EE_L");
JRinit_ctrl = geometricJacobian(robot_model, Q_config, "EE_R");
TLinit_ctrl = Tf_ee_L;
TRinit_ctrl = Tf_ee_R;
robot_model.DataFormat = 'col';
Ginit_ctrl = gravityTorque(robot_model,alg_to_rtbox(Q_init));
%robot_model.DataFormat = 'struct';
%store initial data and a create bus
ctrl_data.JL = JLinit_ctrl;
ctrl_data.JR = JRinit_ctrl;
ctrl_data.TL = TLinit_ctrl;
ctrl_data.TR = TRinit_ctrl;
ctrl_data.G = Ginit_ctrl;

ctrl_data_info = Simulink.Bus.createObject(ctrl_data);
ctrl_data_bus = evalin("base", ctrl_data_info.busName);

%% initial sensor values
[lleg_str, rleg_str, lleg_FS_str, rleg_FS_str] = create_data_struct(QL_init, QR_init, total_mass);
leg_str_info = Simulink.Bus.createObject(lleg_str);
leg_str_bus = evalin('base', leg_str_info.busName);
fs_str_info = Simulink.Bus.createObject(lleg_FS_str);
fs_str_bus = evalin('base', fs_str_info.busName);

%% floating base estimation
% generate cdf's for the algorithm
cdf_data = zmp_cdf(foot,total_mass,0);
cdf_data_info = Simulink.Bus.createObject(cdf_data);
cdf_data_bus = evalin("base", cdf_data_info.busName);