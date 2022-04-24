function [aq] = simple_trajectory(Xini,Xend,Tfin,Ndof,Npoly)
    aq = zeros(Ndof,Npoly);
    for i=1:Ndof
       aq(i,:) = LinTrajGenRed([Tfin,Xini(i),Xend(i),0,0]); 
    end
end

