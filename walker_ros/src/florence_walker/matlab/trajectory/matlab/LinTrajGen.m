function [out] = LinTrajGen(inp)
Tini = inp(1);
Tfin = inp(2);
Xini = inp(3);
Xfin = inp(4);
Vini = inp(5);
Vfin = inp(6);
Aini = inp(7);
Afin = inp(8);

out(1,1) = (2*Tfin^5*Xini - 2*Tini^5*Xfin - Afin*Tfin^2*Tini^5 + 2*Afin*Tfin^3*Tini^4 - Afin*Tfin^4*Tini^3 + Aini*Tfin^3*Tini^4 - 2*Aini*Tfin^4*Tini^3 + Aini*Tfin^5*Tini^2 - 10*Tfin^2*Tini^4*Vfin + 8*Tfin^3*Tini^3*Vfin - 8*Tfin^3*Tini^3*Vini + 10*Tfin^4*Tini^2*Vini - 20*Tfin^2*Tini^3*Xfin + 20*Tfin^3*Tini^2*Xini + 2*Tfin*Tini^5*Vfin - 2*Tfin^5*Tini*Vini + 10*Tfin*Tini^4*Xfin - 10*Tfin^4*Tini*Xini)/(2*(Tfin - Tini)^5);

out(2,1) = (2*Tfin^5*Vini - 2*Tini^5*Vfin - Afin*Tfin^2*Tini^4 - 4*Afin*Tfin^3*Tini^3 + 3*Afin*Tfin^4*Tini^2 - 3*Aini*Tfin^2*Tini^4 + 4*Aini*Tfin^3*Tini^3 + Aini*Tfin^4*Tini^2 + 16*Tfin^2*Tini^3*Vfin - 24*Tfin^3*Tini^2*Vfin + 24*Tfin^2*Tini^3*Vini - 16*Tfin^3*Tini^2*Vini + 60*Tfin^2*Tini^2*Xfin - 60*Tfin^2*Tini^2*Xini + 2*Afin*Tfin*Tini^5 - 2*Aini*Tfin^5*Tini + 10*Tfin*Tini^4*Vfin - 10*Tfin^4*Tini*Vini)/(2*(Tfin - Tini)^5);

out(3,1) = -(Afin*Tini^5 - Aini*Tfin^5 - 8*Afin*Tfin^2*Tini^3 + 8*Aini*Tfin^3*Tini^2 - 12*Tfin^2*Tini^2*Vfin + 12*Tfin^2*Tini^2*Vini + 4*Afin*Tfin*Tini^4 + 3*Afin*Tfin^4*Tini - 3*Aini*Tfin*Tini^4 - 4*Aini*Tfin^4*Tini + 36*Tfin*Tini^3*Vfin - 24*Tfin^3*Tini*Vfin + 24*Tfin*Tini^3*Vini - 36*Tfin^3*Tini*Vini + 60*Tfin*Tini^2*Xfin + 60*Tfin^2*Tini*Xfin - 60*Tfin*Tini^2*Xini - 60*Tfin^2*Tini*Xini)/(2*(Tfin - Tini)^5);

out(4,1) = (Afin*Tfin^4 + 3*Afin*Tini^4 - 3*Aini*Tfin^4 - Aini*Tini^4 - 8*Tfin^3*Vfin - 12*Tfin^3*Vini + 12*Tini^3*Vfin + 8*Tini^3*Vini + 20*Tfin^2*Xfin - 20*Tfin^2*Xini + 20*Tini^2*Xfin - 20*Tini^2*Xini - 8*Afin*Tfin^2*Tini^2 + 8*Aini*Tfin^2*Tini^2 + 80*Tfin*Tini*Xfin - 80*Tfin*Tini*Xini + 4*Afin*Tfin^3*Tini - 4*Aini*Tfin*Tini^3 + 28*Tfin*Tini^2*Vfin - 32*Tfin^2*Tini*Vfin + 32*Tfin*Tini^2*Vini - 28*Tfin^2*Tini*Vini)/(2*(Tfin - Tini)*(Tfin^2 - 2*Tfin*Tini + Tini^2)^2);

out(5,1) = -(30*Tfin*Xfin - 30*Tfin*Xini + 30*Tini*Xfin - 30*Tini*Xini + 2*Afin*Tfin^3 + 3*Afin*Tini^3 - 3*Aini*Tfin^3 - 2*Aini*Tini^3 - 14*Tfin^2*Vfin - 16*Tfin^2*Vini + 16*Tini^2*Vfin + 14*Tini^2*Vini - 2*Tfin*Tini*Vfin + 2*Tfin*Tini*Vini - 4*Afin*Tfin*Tini^2 - Afin*Tfin^2*Tini + Aini*Tfin*Tini^2 + 4*Aini*Tfin^2*Tini)/(2*(Tfin - Tini)*(Tfin^4 - 4*Tfin^3*Tini - 4*Tfin*Tini^3 + Tini^4 + 6*Tfin^2*Tini^2));

out(6,1) = (12*Xfin - 12*Xini - 6*Tfin*Vfin - 6*Tfin*Vini + 6*Tini*Vfin + 6*Tini*Vini + Afin*Tfin^2 + Afin*Tini^2 - Aini*Tfin^2 - Aini*Tini^2 - 2*Afin*Tfin*Tini + 2*Aini*Tfin*Tini)/(2*(Tfin - Tini)^2*(3*Tfin*Tini^2 - 3*Tfin^2*Tini + Tfin^3 - Tini^3));

end
