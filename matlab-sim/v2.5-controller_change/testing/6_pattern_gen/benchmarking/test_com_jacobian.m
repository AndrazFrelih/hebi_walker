use_zeros = 1;
if ~use_zeros
    QLtest = QL_home';
    QRtest = QR_home';
else
    QLtest = zeros(6,1);
    QRtest = zeros(6,1);
end
Qtest = alg_to_rtbox([QLtest;QRtest]);
Qtest_conf= struct('JointName',{Q_config.JointName}','JointPosition',num2cell(Qtest));

tic
%from derived model
HlistCOM = zeros(13,4,4);
HlistCOM(1,:,:) = Hbase_com;
Hlist_L = H_abs_bL(QLtest);
HlistCOM(2:7,:,:) = H_abs_bLcom(Hlist_L,QLtest);
Hlist_R = H_abs_bR(QRtest);
HlistCOM(8:end,:,:) = H_abs_bRcom(Hlist_R,QRtest);

HlistAbs = zeros(size(Hlist_L,1)+size(Hlist_R,1),4,4);
HlistAbs(1:end/2,:,:) = Hlist_L;
HlistAbs(end/2+1:end,:,:) = Hlist_R;

Jlist = compute_com_jac_base(12,HlistAbs,HlistCOM);
toc

JeeLder = reshape(Jlist(7,:,1:end/2),6,6);
JeeRder = reshape(Jlist(13,:,end/2+1:end),6,6);

% from Rtoolbox
JeeL = geometricJacobian(robot_model,Qtest_conf,frame_list.coms.lleg(6));
JeeR = geometricJacobian(robot_model,Qtest_conf,frame_list.coms.rleg(6));
Jc = JeeL + JeeR;
Jalg_inv = [Jc(:,1:2:end),Jc(:,2:2:end)];
Jalg = [
    Jalg_inv(4:6,:);
    Jalg_inv(1:3,:);
];
JeeLalg = Jalg(:,1:end/2);
JeeRalg = Jalg(:,end/2+1:end);

% BOTH VERSIONS COMPUTE SAME VALUES