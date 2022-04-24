function [R] = direct_fb_estimation(Rold, a, w, dt)
    %gyro estimate
    Phi = norm(w)*dt;
    if Phi==0
        Rw = Rold;
    else
        Rw = Rold*(eye(3) + sin(Phi)/Phi * skew_mat(w) + (1-cos(Phi))/Phi^2 * skew_mat(w) * skew_mat(w));
    end
    
    %accelerometer estimate
    g = [0,0,1];
    an = a/norm(a);
    v = cross(an,g);
    c = dot(an,g);
    Ra = eye(3) + skew_mat(v) + 1/(1+c)*skew_mat(v)*skew_mat(v);
    
    %get euler angles
    eul_w = rotm2eul(Rw);
    eul_a = rotm2eul(Ra);
    
    %compute weights
    alpha = 0.5;
    Ww = [
        1, alpha, alpha
    ];
    Wa = [
        0, 1 - alpha, 1 - alpha
    ];
    
    %fuse both measurements
    eul_fused =  Ww .* eul_w +  Wa .* eul_a;
    R = eul2rotm(eul_fused);
end

