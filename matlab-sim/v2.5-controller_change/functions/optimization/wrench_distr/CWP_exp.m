function [U] = CWP_exp(param,cont_fz)
    X = param(1);
    Y = param(2);
    mu = param(3);
    k = mu*(X+Y);
    
    if cont_fz
        C = -1;
    else
        C = 1;
    end
    
    U = [
        +1, 0, -mu*C, 0, 0, 0;
        -1, 0, -mu*C, 0, 0, 0;
        0, +1, -mu*C, 0, 0, 0;
        0, -1, -mu*C, 0, 0, 0;
        0, 0, -Y*C, +1, 0, 0;
        0, 0, -Y*C, -1, 0, 0;
        0, 0, -X*C, 0, +1, 0;
        0, 0, -X*C, 0, -1, 0;
        +Y, +X, -k*C, -mu, -mu, -1;
        -Y, +X, -k*C, +mu, -mu, -1;
        +Y, -X, -k*C, -mu, +mu, -1;
        -Y, -X, -k*C, +mu, +mu, -1;
        +Y, +X, -k*C, +mu, +mu, +1;
        -Y, +X, -k*C, -mu, +mu, +1;
        +Y, -X, -k*C, +mu, -mu, +1;
        -Y, -X, -k*C, -mu, -mu, +1;
    ];
end

