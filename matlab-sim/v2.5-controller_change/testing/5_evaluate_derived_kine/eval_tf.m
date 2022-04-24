QLtest = ones(6,1);
QRtest = ones(6,1);
QLtest = QL_home';
QRtest = QR_home';

Qtest = zeros(12,1);
Qtest(1:2:end) = QLtest;
Qtest(2:2:end) = QRtest;

Qtest_conf= struct('JointName',{Q_config.JointName}','JointPosition',num2cell(Qtest));

tic
[Hlist_L, Hlist_R] = get_transforms_B(QLtest,QRtest);
toc

FR = 7;
HLee_der = reshape(Hlist_L(FR,:,:),4,4)

HLee_rt = getTransform(robot_model,Qtest_conf,frame_list.links.lleg(FR),frame_list.links.base)

HRee_der = reshape(Hlist_R(FR,:,:),4,4)

HRee_rt = getTransform(robot_model,Qtest_conf,frame_list.links.rleg(FR),frame_list.links.base)

%Transformations are confirmed correct!!