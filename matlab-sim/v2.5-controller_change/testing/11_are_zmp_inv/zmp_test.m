R = eye(3);
t = ones(3,1);
Fe =[10, 5, 10, 2]';

Forig1 = Fe;

Porig1 = foot.fsen.Pedge;
Ptarg1 = foot.fsen.Pmeas;

AinvX1 = foot.zmp.Ae.xinv;
AinvY1 = foot.zmp.Ae.yinv;
Apseud1 = foot.zmp.Ae.finv;

%transformation from edge to measurement (SENSOR)
[Ftarg1,zmp_glo1,zmp_loc1,~] = force_remapping(Forig1,AinvX1,AinvY1,Apseud1,Porig1,Ptarg1, R, t, 0,'o');

Forig2 = Ftarg1;

Porig2 = foot.fsen.Pmeas;
Ptarg2 = foot.fsen.Pedge;

AinvX2 = foot.zmp.Am.xinv;
AinvY2 = foot.zmp.Am.yinv;
Apseud2 = foot.zmp.Am.finv;

%transformation from measurement to edge (CONTROLLER)
[Ftarg2,zmp_glo2,zmp_loc2,~] = force_remapping(Forig2,AinvX2,AinvY2,Apseud2,Porig2,Ptarg2, R, t, 0,'t');

%% JIELDS SAME RESULTS - SHOULD BE A VALID SOLUTION
zmp_glo1
zmp_glo2
zmp_loc1
zmp_loc2