Ndof = int32(12);
Npoly = int32(6);

qini = zeros(Ndof,1);
qend = zeros(Ndof,1);
Tfin = 2;
%calculate polynomial coefficients
aq = simple_trajectory(qini,qend,Tfin,Ndof,Npoly);

