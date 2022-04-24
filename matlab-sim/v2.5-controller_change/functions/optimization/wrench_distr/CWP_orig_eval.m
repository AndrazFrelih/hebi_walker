function [ineq] = CWP_orig_eval(w,param)
    X = param(1);
    Y = param(2);
    mu = param(3);
    k = mu*(X+Y);
    
    fx = w(1);
    fy = w(2);
    fz = w(3);
    tx = w(4);
    ty = w(5);
    tz = w(6);
    tzmin =  - k*fz + abs(Y*fx - mu*tx) + abs(X*fy - mu*ty);
    tzmax =  + k*fz - abs(Y*fx + mu*tx) - abs(X*fy + mu*ty);
    
    ineq = zeros(6,1);
    
    ineq(1) = abs(fx)-mu*fz;
    ineq(2) = abs(fy)-mu*fz;
    ineq(3) = abs(tx)-Y*fz;
    ineq(4) = abs(ty)-X*fz;
    ineq(5) = - tz + tzmin;
    ineq(6) = + tz - tzmax;
end

