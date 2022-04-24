function [J] = compute_jacobian2(Hlist,frame,Hend)
    %ACTUALLY THE PROXIMAL VARIATION OF JACOBIAN COMPUTATION (THIS IS WHY
    %ONE MORE TRANSFORMATION IS NEEDED THAN THERE ARE DOFs)

    %Hlist is a list of absolute frames
    s = size(Hlist);
    Ndof_leg = max(s)-1;
    Ndof = 6;
    tend = reshape(Hend(1:3,4),3,1);
    J = zeros(Ndof,Ndof_leg);
    for j=1:Ndof_leg
        if(j<=frame)
            Hstart = reshape(Hlist(j,:,:),4,4);
            tstart = reshape(Hstart(1:3,4),3,1);
            Zcf = reshape(Hstart(1:3,3),3,1);
            Jvij = cross(Zcf,tend - tstart);
            Jwij = Zcf;
            Jij = [Jvij;Jwij];
        else
            %later links in the chain, do not contribute to the velocity of this
            %link (kinematically speaking)
            Jij = sym(zeros(6,1)); 
        end
        J(:,j) = Jij;
    end
end

