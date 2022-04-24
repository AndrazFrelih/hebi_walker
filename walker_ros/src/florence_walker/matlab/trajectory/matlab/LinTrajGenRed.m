function [out] = LinTrajGenRed(inp)
Tfin = inp(1);
Xini = inp(2);
Xfin = inp(3);
Vini = inp(4);
Vfin = inp(5);

out = zeros(6,1);

out(1) = Xini;

out(2) = Vini;

out(3) = 0;

out(4) = -(2*(5*Xini - 5*Xfin + 2*Tfin*Vfin + 3*Tfin*Vini))/Tfin^3;

out(5) = (15*Xini - 15*Xfin + 7*Tfin*Vfin + 8*Tfin*Vini)/Tfin^4;

out(6) = -(3*(2*Xini - 2*Xfin + Tfin*Vfin + Tfin*Vini))/Tfin^5;

end
