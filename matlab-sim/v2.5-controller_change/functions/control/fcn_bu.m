function [TL,TR,p_des,p_ref] = fcn_bu(time, MSL,LFL,MSR,LFR, ZMP_ref, COM_ref, ctrl_data, ctrl_config, twb, Rwb, GenMotion)
%backup of the main controller function

persistent dcm_int;
persistent init_corr corr_stored;

if isempty(dcm_int)
    dcm_int = zeros(2,1); %initialise the integrator
    corr_stored = zeros(3,1);
end

%% constants & configuration

z_des = ctrl_config.com_height;
g = 9.81;
omega = sqrt(g/z_des);
zp_des = 0;
Mtot = ctrl_config.mass;
foot_dims = ctrl_config.foot_dims*0.99;
K = Mtot*omega^2;

%% values from the pattern generator
phase = GenMotion(1);
WshiftL = GenMotion(2);
WshiftR = GenMotion(3);
ZMP_des = GenMotion(4:5);
COM_des = GenMotion(6:7);
COMp_des = GenMotion(8:9);
qL_des = GenMotion(10:15);
qR_des = GenMotion(16:21);
qLp_des = GenMotion(22:27);
qLr_des = GenMotion(28:33);

%% Desired state values
%desired joint angles
qLd = qL_des;%qLd = ctrl_config.qLd;
qRd = qR_des;%qRd = ctrl_config.qRd;
%desired joint velocities
qLpd = qLp_des;
qRpd = qLr_des;
% desired ZMP
dcm_offs = [0;0];
p_des = ZMP_des; %p_des = ctrl_config.pdes_ini(1:2) + dcm_offs;
% desired COM
C_d = COM_des;%C_d = p_des;zeros(3,1);
Cp_d = COMp_des;%Cp_d = [0,0]';
% desired DCM
dcm_des = C_d + Cp_d/omega;


%% Measurements from motors
[LL,RL] = group_motor_data(MSL,MSR);
qL = LL.q;
qR = RL.q;
qLp = LL.qp;
qRp = RL.qp;

%% COM & ZMP observer (Now: reference data - Todo: implment an observer)
C_meas = COM_ref(1:3);
Cp_meas = COM_ref(4:6);
p_meas = ZMP_ref;


%% COM controller 
% gain setup
sc = omega;
Kp_com = 3.5*[sc;sc];
Kp_comz = 500*sc;
Kd_com = 100*sc;
Kp_p = 0.99*[sc;sc];

% leaky integrator
Ki_com = 2*[sc;sc];
Kleak = [0.1;0.1];



% control law for COM forces
if C_meas(3)==0
    czerr = 0;
    C_meas_ada = C_meas;
else
    if isempty(init_corr)
        init_corr = 1;
        corr_stored = C_meas - [0;0;z_des];
    end
    C_meas_ada = C_meas - [0,0,corr_stored(3)]'; %this will have to be performed in practice as well
    
    czerr = Kd_com*(zp_des - Cp_meas(3)) + Kp_comz*(z_des-C_meas_ada(3));
end

%% DCM observer
dcm_meas = C_meas_ada(1:2) + C_meas_ada(1:2)/omega;
% integrate the DCM error (IIR filter)
dT = 1/ctrl_config.fctrl;
dcm_err = dcm_des - dcm_meas; %integral error
d_err = dcm_err - Kleak.*dcm_int; %inclusion of leakage
dcm_int = dcm_int + dT * d_err;

%% CONTROLLER
% reference value for the ZMP
p_ref = p_des -(1+Kp_com./omega).*dcm_err - Ki_com./omega.*dcm_int + Kp_p./omega.*(p_des-p_meas);



Fd = [
    -K*(C_meas_ada(1)-p_ref(1));
    -K*(C_meas_ada(2)-p_ref(2));
    -(Mtot*g + czerr);
];

% control law for torques around COM
% angular velocities measured in motors attached rigidly to base (M1 - L&R)
%left measurement
w_1L = LL.w(:,1);
R_base_IMU_1L = rotz(-pi/6);
wbaseL = R_base_IMU_1L*w_1L;
%right measurement
w_1R = RL.w(:,1);
R_base_IMU_1R = rotz(+pi/6);
wbaseR = R_base_IMU_1R*w_1R;
%average both
omega=(wbaseL+wbaseR)/2;
Td = floating_base_orientation(Rwb',omega); %transpose the rotm to get the correct mapping (FROM:desired TO:actual)
%Td = zeros(3,1);
% generate an overall wrench (in the world frame)
Wc = [Fd;Td]; 

% get jaccobians 
% WATCH OUT => First lines 1-3 are Jw and 4-6 are Jv (swapped rot. and tr.
% motion)
JL = ctrl_data.JL(:,1:2:end); 
JLcorr = [JL(4:6,:);JL(1:3,:)];

JR = ctrl_data.JR(:,2:2:end);
JRcorr = [JR(4:6,:);JR(1:3,:)];

% get transformations (feet to base)
TF_world_base = [
    Rwb, twb;
    zeros(1,3),1
];

TF_world_com = [
    eye(3),C_meas;
    zeros(1,3),1
];

TF_com_base = TF_world_com\TF_world_base; %TF from base to COM

TF_eeL_base = ctrl_data.TL; %TF from left end effector to base
TF_eeR_base = ctrl_data.TR; %TF from right end effector to base

% get required wrench mappings
AdT_gcb = Ad_g(TF_com_base(1:3,1:3),TF_com_base(1:3,4))';
AdT_gbl = Ad_g(TF_eeL_base(1:3,1:3),TF_eeL_base(1:3,4))';
AdT_gbr = Ad_g(TF_eeR_base(1:3,1:3),TF_eeR_base(1:3,4))';

%% torque distribution
% transform the wrench from com to the body frame (gcb transforms vectors
% from base to com, but AdT is dual - transforms in the oposite order)
%Wb = AdT_gcb*Wc;
% equally distribute the wrench and transform them to the feet frames
%Wlfoot = AdT_gbl*Wb /2;
%Wrfoot = AdT_gbr*Wb /2;
[Wlfoot,Wrfoot] = distribute_wrench_tracking(Wc,JLcorr,JRcorr,TF_com_base,TF_eeL_base,TF_eeR_base,foot_dims,0.6,0,phase,WshiftL,WshiftR);
% map the wrenches to respective joint torques
TLtc = JLcorr'*Wlfoot;
TRtc = JRcorr'*Wrfoot;


%% Test if the zmp recomputation matters
TF_world_eeL = TF_world_base*inv(TF_eeL_base);
TF_world_eeR = TF_world_base*inv(TF_eeR_base);
RwL = TF_world_eeL(1:3,1:3);
RwR = TF_world_eeR(1:3,1:3);
twL = TF_world_eeL(1:3,4);
twR = TF_world_eeR(1:3,4);
[pL,Wlfoot_rot] = zmp_from_wrench(Wlfoot,RwL,twL);
[pR,Wrfoot_rot] = zmp_from_wrench(Wrfoot,RwR,twR);
p_rec = (Wlfoot_rot(3)*pL+Wrfoot_rot(3)*pR)/(Wlfoot_rot(3)+Wrfoot_rot(3));

%% Joint control (secondary task)
%Kp = [20,150,150,100,20,4]';
%Kd = [5,30,60,20,5,1]';
Kp = [40,250,350,250,40,4]';
Kd = [10,40,60,20,10,1]';

qLpr = qLp + Kp.*(qLd-qL);
qRpr = qRp + Kp.*(qRd-qR);

%separated pd control
TLpos = (Kp.*(qLd-qL));
TRpos = (Kp.*(qRd-qR));
TLvel = (Kd.*(qLpd-qLp));
TRvel = (Kd.*(qRpd-qRp));

% Null space projector (for a 2 level hierarchy, successive and augmented projectors are the same)
use_NS_ctrl = 2;
if use_NS_ctrl && phase == 0 %can only be used in DS phase
    rho = 0.1; %needs to be included
    Jfull = [JLcorr,JRcorr];
    JLpseud = pseudoinv(Jfull,rho,'right');
    % compute the projector
    Nproj2 = (eye(12)-Jfull'*JLpseud');
    % project the secondary task to the nullspace of the primary one
    if use_NS_ctrl==1
        %only position is projected 
        Tsec = Nproj2*[TLpos;TRpos];
        TLsec = Tsec(1:end/2);
        TRsec = Tsec(end/2+1:end);
        %velocity is added
        TLsec = TLsec + TLvel;
        TRsec = TRsec + TLvel;
    elseif use_NS_ctrl==2
        %position and velocity are projected
        Tsec = Nproj2*[TLpos+TLvel;TRpos+TRvel];
        TLsec = Tsec(1:end/2);
        TRsec = Tsec(end/2+1:end);
    end
else
    % simply add the secondary task to the primary on
    TLsec = TLpos + TLvel;
    TRsec = TRpos + TLvel;
    if phase == 1
        Grem = rtbox_to_alg(ctrl_data.G);
        Gl = Grem(1:end/2);
        Gr = Grem(end/2+1:end);
        if WshiftL>WshiftR
            %right leg is swinging
            TRsec = TRsec + Gr;
        else
            %left leg is swinging
            TLsec = TLsec + Gl;
        end
    end
end


    

%% Combined control
TL = TLtc + TLsec;
TR = TRtc + TRsec;


