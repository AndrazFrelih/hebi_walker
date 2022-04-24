function [Tcom] = floating_base_orientation(R,w)
    % Implements an orientation control law from:
    % Henze et. al.: Passivity based whole body balancing for torque controlled
    % humanoid robots in multi contact scenarios

    % Params:
    % R - rotation matrix mapping from the desired coordinate frame
    % orientation to the actual orientation of the floating base*
    %   FROM: desired frame
    %   TO: actual frame
    % w - rotational velocity vector

    %% virtual spring-damper system
    %stiffness matrix
    Eps0 = 1000;
    Eps = Eps0*eye(3);
    % damping matrix
    D0 = 1;
    D = D0*eye(3);

    % prepare the quaternion
    q = rotm2quat(R);
    q_s = q(1);
    q_v = q(2:end)';

    % compute the stiffness induced torque:
    Ts = -2*R*(q_s*Eps*q_v + cross(q_v,Eps*q_v));

    % compute the damping induced torque:
    Td = -D*w;

    %resultant torque
    Tcom = Ts + Td;

end

