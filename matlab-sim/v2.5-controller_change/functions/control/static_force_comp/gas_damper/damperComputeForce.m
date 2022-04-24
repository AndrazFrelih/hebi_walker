function [F] = damperComputeForce(Hhip,Hknee,consts)
    % attachment points
    rhip = Hhip(1:3,4);
    rknee = Hknee(1:3,4);
    % vector between gas spring attachment points
    vec = -rknee + rhip;
    % distance between attachment points 
    dist = norm(vec);
    % direction in which the force is acting
    dir = vec/dist;
    % amplitude of the force
    Fa = damperForceModel(dist,consts);
    % force vector
    F = dir*Fa;
end

