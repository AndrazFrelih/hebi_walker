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

[Hlist_L] = H_abs_bL(QLtest);
[Hlist_R] = H_abs_bR(QRtest);
%% consistency proof
%for all frames (1-7) the derived jacobian is the same as the one from the
%toolbox (angles have to be zero, as the toolbox is using the proximal frame
%notation)
frame = 7;

% (1) FASTEST - CORRECT
tic
JLalg = compute_jacobian(Hlist_L(1:end,:,:),frame);
toc
robot_obj{9}.in.H
% (2) SECOND FASTEST - INCORRECT (could be corrected as it uses the same algo
% as (1) ) => Bad implementation, deleted
%tic
%JLst = Jb6L(QL_home);
%toc

% (3) SLOWEST - CORRECT
tic
JLrt_t1 = geometricJacobian(robot_model,Qtest_conf,frame_list.links.lleg(frame));
JLrt_t1 = JLrt_t1(:,1:2:end);
JLrt_t1 = [JLrt_t1(4:6,:);JLrt_t1(1:3,:)];
toc

JLalg;
%JLst;
JLrt_t1;

%difference between jacobians
disp(round(1e9*(JLalg-JLrt_t1))/1e9);

%% get the null space projector
JLrt = geometricJacobian(robot_model,Qtest_conf,"EE_L");
JLrt = JLrt(:,1:2:end);
JLrt = [JLrt(4:6,:);JLrt(1:3,:)];

JRrt = geometricJacobian(robot_model,Qtest_conf,"EE_R");
JRrt = JRrt(:,2:2:end);
JRrt = [JRrt(4:6,:);JRrt(1:3,:)];

Jfull = [JLrt,JRrt];
JLpseud = pseudoinv(Jfull,0.01,'right');
Pr = (eye(12)-Jfull'*JLpseud');
Pr2 = Pr*Pr;

disp("Difference abs(P-P*P) =" + sum(sum(abs(Pr-Pr2))));
