function [p,w_rot] = zmp_from_wrench(w, Rw, tw)
    f = Rw*w(1:3);
    T = Rw*w(4:6);

    fx = f(1);
    fy = f(2);
    fz = f(3);
    Tx = T(1);
    Ty = T(2);
    
    pz = 0;
    
    px = (-Ty-(tw(3)-pz)*fx+tw(1)*fz)/fz;
    py = (+Tx-(tw(3)-pz)*fy+tw(2)*fz)/fz;
    p = [px;py];
    
    w_rot = [f;T];
end

