qB = zeros(6,1);

use_zeros = 0;
if ~use_zeros
    QLtest = QL_home';
    QRtest = QR_home';
else
    QLtest = zeros(6,1);
    QRtest = zeros(6,1);
end
Qtest = alg_to_rtbox([QLtest;QRtest]);
Qtest_conf= struct('JointName',{Q_config.JointName}','JointPosition',num2cell(Qtest));

%relatively fast
robot_model.DataFormat = 'column';
tic
C1 = centerOfMass(robot_model,Qtest);
M1 = toc;
robot_model.DataFormat = 'struct';

%this is super fast
tic
Hlist_2 = zeros(13,4,4);
Hlist_2(1,:,:) = Hbase_com;
Hlist_L = H_abs_bL(QLtest);
Hlist_2(2:7,:,:) = H_abs_bLcom(Hlist_L,QLtest);
Hlist_R = H_abs_bR(QRtest);
Hlist_2(8:end,:,:) = H_abs_bRcom(Hlist_R,QRtest);
M2 = toc;

%slightly faster than C1
tic
C2 = compute_com_pos(Hlist_2,mlist);
M2 = M2 + toc;

%much faster than Hlist_2 (the problem of Hlist_2 is that is not computed
%iteratively)
tic
Hlist_1 = zeros(13,4,4);
Hlist_1(1,:,:) = getTransform(robot_model,Qtest_conf,frame_list.coms.base,frame_list.links.base);
Ntf = 6;
for i=1:Ntf
    Hlist_1(i+1,:,:) = getTransform(robot_model,Qtest_conf,frame_list.coms.lleg(i),frame_list.links.base);
    Hlist_1(i+1+Ntf,:,:) = getTransform(robot_model,Qtest_conf,frame_list.coms.rleg(i),frame_list.links.base);
end
M1 = M1 + toc;

disp("Robotic toolbox takes " + M1 +"s to compute the COM and get all the transformations");
disp("Direct method takes " + M2 +"s to compute the COM and get all the transformations");

for i=1:size(Hlist_1,1)
    H1 = reshape(Hlist_1(i,:,:),4,4);
    H2 = reshape(Hlist_2(i,:,:),4,4);
    disp("Transformation "+i+" is:");
    disp(H1);
    disp(H2);
    disp("Err = " + sum(sum(abs(H1-H2))));
    input("press Enter to continue");
end


