function [X,Xp] = traj_gen(t)
    T0 = 1;
    omega = 2*pi/T0;
    z0 = 0.05;
    zp0 = z0*omega;
    z = z0*sin(omega*t);
    zp = zp0*sin(omega*t);
    
    X = [
        0;0;z;
    ];

    Xp = [
        0;0;zp;
    ];
    
end

