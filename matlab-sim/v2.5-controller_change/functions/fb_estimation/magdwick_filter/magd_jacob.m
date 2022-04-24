function [J] = magd_jacob(q,d)
    q1 = q(1);
    q2 = q(2);
    q3 = q(3);
    q4 = q(4);
    dx = d(1);
    dy = d(2);
    dz = d(3);
    J11 = 2*dy*q4 - 2*dz*q3;
    J12 = 2*dy*q3 + 2*dz*q4;
    J13 = -4*dx*q3 + 2*dy*q2 - 2*dz*q1;
    J14 = -4*dx*q4 + 2*dy*q1 + 2*dz*q2;
    J21 = -2*dx*q4 + 2*dz*q2;
    J22 = 2*dx*q3 - 4*dy*q2 + 2*dz*q1;
    J23 = 2*dx*q2 + 2*dz*q4;
    J24 = -2*dx*q1 - 4*dy*q4 + 2*dz*q3;
    J31 = 2*dx*q3 - 2*dy*q2;
    J32 = 2*dx*q4 - 2*dy*q1 - 4*dz*q2;
    J33 = 2*dx*q1 + 2*dy*q4 - 4*dz*q3;
    J34 = 2*dx*q2 + 2*dy*q3;
    
    J = [
        J11,J12,J13,J14;
        J21,J22,J23,J24;
        J31,J32,J33,J34;
    ];
end

