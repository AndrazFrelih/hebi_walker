function [Targ, ctrl_on_out] = foot_damping_ctrl(zloc,Z1,Z2,Z3,ctrl_on)    
    
    % saturate the zmp to get the desired one
    z_m = zloc(1:2);
    z_d = saturate_zmp(z_m,Z1);

    % get zmp error
    err_zmp = z_d - z_m;
    
    % check on which sets zmp lays
    Ze = abs(z_m);
    
    e1 = all(Ze<Z1);
    e2 = ~e1 && all(Ze<Z2);
    e3 = ~e2 && all(Ze<Z3);
    
    % gains
    Acopx = 1;
    Acopy = 1;
    Kcop = [
        Acopx, 0;
        0,     Acopy;
    ];

    % ZMP tracking
    % rotational velocities of feet
    w_foot = Kcop * err_zmp;

    Targ = zeros(3,1);
    ctrl_on_out = ctrl_on;
    if e3
        % if the zmp is in zone 3, turn on the controller
        ctrl_on_out = 1;
    end
    
    if ctrl_on_out && ~e1
        % if the controller is on and zmp is not in zone 1, compute the target
        Targ(1:2) = w_foot;
    else
        % if the zmp is in zone one or the controller is off and zmp is in
        % zone 2, then set the controller to off
        ctrl_on_out = 0;
    end
end

