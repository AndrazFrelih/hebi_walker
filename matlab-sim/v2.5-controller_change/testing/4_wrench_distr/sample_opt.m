

%% Constraints

% foot size
X = foot.dims(1)/2;
Y = foot.dims(2)/2;
% coulomb friction
mu = 0.4;
% parameters of the contact wrench pyramid (CWP)
param = [X;Y;mu];

% investigate if there is any deviation between the original and extended
% constraint formulation (6 abs. val. ineq. constr. => 16 lin. ineq. constr.)
num_of_discr = eval_form_eq(100000,param,30);
if num_of_discr
    disp("Constraint conversion was not done properly.");
else
    disp("Linear constraints properly describe absolute value constraints.");
end

U = CWP_exp(param,0);
%for double support, zmp checking should be adapted
U = [U(1:4,:);U(9:end,:)]; 

%separately consider the z-force (as it should have a threshold)
pmin = 5;
efz = [0;0;1;0;0;0];

%% Transformations

% get COM position in base frame
robot_model.DataFormat = 'row'; %necessary for dynamic computations
Com = centerOfMass(robot_model, [Q_config.JointPosition])';
robot_model.DataFormat = 'struct'; %change it back

% COM to base transformation
TF_base_com = [
    eye(3),Com;
    zeros(1,3),1
];
TF_com_base = inv(TF_base_com); %vectors are transformed from base frame to com frame
AdT_base_com = Ad_g2(TF_com_base)'; %forces are transformed from com frame to base (duality)

% feet transformations
Tf_com_lf = TF_base_com\getTransform(robot_model,Q_config,frame_list.links.lleg(7),frame_list.links.base);
AdT_com_lf = Ad_g2(Tf_com_lf)';
Tf_com_rf = TF_base_com\getTransform(robot_model,Q_config,frame_list.links.rleg(7),frame_list.links.base);
AdT_com_rf = Ad_g2(Tf_com_rf)';

% ankle transformations
Tf_com_la = TF_base_com\getTransform(robot_model,Q_config,frame_list.links.lleg(6),frame_list.links.base);
AdT_com_la = Ad_g2(Tf_com_la)';
Tf_com_ra = TF_base_com\getTransform(robot_model,Q_config,frame_list.links.rleg(6),frame_list.links.base);
AdT_com_ra = Ad_g2(Tf_com_ra)';

% get jaccobians
% left
J_lf_full = geometricJacobian(robot_model,Q_config,frame_list.links.lleg(7));
J_lf = J_lf_full(:,1:2:end);
J_lf = [
    J_lf(4:6,:);
    J_lf(1:3,:)
];
% right
J_rf_full = geometricJacobian(robot_model,Q_config,frame_list.links.rleg(7));
J_rf = J_rf_full(:,2:2:end);
J_rf = [
    J_rf(4:6,:);
    J_rf(1:3,:)
];

% transform constraints
UL = U*AdT_com_lf;
UR = U*AdT_com_rf;
efzL = efz'*AdT_com_lf;
efzR = efz'*AdT_com_rf;
JL = J_lf'*AdT_com_lf;
JR = J_rf'*AdT_com_rf;

%% Optimization problem

%penalise certain torques
Wjoint = eye(6);
Wjoint(1,1) = 0.1;
Wjoint(2,2) = 0.001;
Wjoint(3,3) = 0.001;
Wjoint(4,4) = 1;
Wjoint(5,5) = 100;
Wjoint(6,6) = 1;
%Wjoint = zeros(6);

%weights
A1 = 1000;
A2 = 1;
J = @(w,wdes) A1 * sq_mat2(wdes-(w(1:end/2)+w(end/2+1:end))) + A2*(sq_mat(JL'*w(1:end/2),Wjoint)+sq_mat(JR'*w(end/2+1:end),Wjoint));

wdes = [1,0,250,0,0,0]';
Jopt = @(w)J(w,wdes);

w0 = [wdes/2; wdes/2];

%ineq. constraints
A = [
    UL, zeros(size(UL));
    zeros(size(UR)), UR;
    -efzL, zeros(size(efzL));
    zeros(size(efzR)), -efzR;
];
b = [
    zeros(max(size(UL)),1);
    zeros(max(size(UR)),1);
    -pmin
    -pmin
];

options = optimoptions('fmincon','Algorithm','sqp','Display','none');
[wsol,Jval] = fmincon(Jopt,w0,A,b,[],[],[],[],[],options);
wLleg = wsol(1:6);
wRleg = wsol(7:end);
disp(wLleg+wRleg);

T_L = JL'*wLleg;
T_R = JR'*wRleg;

T_Leq = JL'*wdes/2;
T_Req = JR'*wdes/2;