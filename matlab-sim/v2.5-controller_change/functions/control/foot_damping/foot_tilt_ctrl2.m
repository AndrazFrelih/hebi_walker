function [qdes_new,qpdes_new] = foot_tilt_ctrl2(FmL, FmR, qdes, qpdes, dt, active)
    persistent dq;
    if isempty(dq)
       dq = zeros(4,1); 
    end
    qdes_ada = qdes(7:end);
    qpdes_ada = qpdes(7:end);
    qLdes = qdes_ada(1:end/2);
    qRdes = qdes_ada(end/2+1:end);
    qLp_des = qpdes_ada(1:end/2);
    qRp_des = qpdes_ada(end/2+1:end);
    
    if active
        K1 = 0.003;
        K2 = 0.003;
        Kleak = 0.2/dt; %leakage parameter can go from 0 (integrator) to 1/dt (direct feedthrough)

        Fleg = @(Fm) [
            (Fm(1)+Fm(2))/2; %front
            (Fm(3)+Fm(4))/2; %back
            (Fm(1)+Fm(4))/2; %left
            (Fm(2)+Fm(3))/2; %right
            
        ];

        FL = Fleg(FmL);
        FR = Fleg(FmR);

        uL = [
            K1*(+FL(1)-FL(2));
            K2*(-FL(3)+FL(4));
        ];

        uR = [
            K1*(-FR(1)+FR(2));
            K2*(-FR(3)+FR(4));
        ];

        pmin = 5;

        if all(FL>pmin)
            qpL = - Kleak*dq(1:2);
        else
            qpL = uL - Kleak*dq(1:2);
        end

        if all(FR>pmin)
            qpR = - Kleak*dq(3:4);        
        else
            qpR = uR - Kleak*dq(3:4);
        end

        qp = [qpL;qpR];

        dq = dq + dt*qp;

        qLdes_new = qLdes + [zeros(4,1);dq(1:2)];
        qRdes_new = qRdes + [zeros(4,1);dq(3:4)];
        qLp_des_new = qLp_des + [zeros(4,1);qp(1:2)];
        qRp_des_new = qRp_des + [zeros(4,1);qp(3:4)];
    else
        qLdes_new = qLdes;
        qRdes_new = qRdes;
        qLp_des_new = qLp_des;
        qRp_des_new = qRp_des;
    end
    
    qdes_new = [qdes(1:6);qLdes_new;qRdes_new];
    %qpdes_new = [qpdes(1:6);qLp_des_new;qRp_des_new];
    qpdes_new = qpdes;
end

