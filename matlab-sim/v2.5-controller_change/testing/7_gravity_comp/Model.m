function [Model] = Model(robot, t,y, Q_init)
    Kp = [20,70,50,30,30,1]'*0.5;
    KP = [Kp;Kp];
    Kd = [1,5,10,10,5,1]'*0.2;
    KD = [Kd;Kd];
 
    Tctrl =  KP.*(Q_init-y(1:end/2)) + KD.*-y(end/2+1:end);
    Tcomp = gravityTorque(robot,y(1:end/2))*0.95;
    Ttot = Tctrl + Tcomp;
    Model = [
        y(end/2+1:end);
        forwardDynamics(robot,y(1:end/2),y(end/2+1:end),Ttot);
    ];

    if mod(t,0.02) == 0
        show(robot, y(1:end/2));
        pause(0.2);
    end

end

