function [xfb,qL,qR] = initial_config(xinit,qinit, com, xL, xR, mlist)
    %% calculate the initial pose of the robot:
    % first a statically stable initial state is computed
    C_des = com;

    % left foot task
    XLdes = [
        xL(1);          %initial X-position for center of foot
        xL(2);          %initial Y-position for center of foot
        zeros(4,1);     %initial Z-position for all vertices
    ];

    % right foot task
    XRdes = [
        xR(1);          %initial X-position for center of foot
        xR(2);          %initial Y-position for center of foot
        zeros(4,1);     %initial Z-position for all vertices
    ];

    % initial search value for the base position and joint angles
    zinit = [xinit;qinit];
    Nz = size(zinit,1); %dimensionality of the initial problem
    Nq = size(qinit,1); %dimensionality of the joint space


    %desired angles
    Qdes = qinit;

    % cost function
    w1 = 1000;
    w2 = 0.01;
    cost_fcn = @(z) (w1*sq_mat2(get_com_error(z(1:3),z(4:9),z(10:15),C_des,mlist))+w2*sq_mat2(Qdes-z(4:15)));

    % nonlinear equality constraint
    nlcstr_fcn = @(z)nlcstr_init(z(1:3),z(4:9),z(10:15),XLdes,XRdes);

    % optimization options
    %options = optimoptions('fmincon','Algorithm','interior-point','Display','none');
    options = optimoptions('fmincon','Algorithm','sqp','Display','none');

    % solve the optimization
    %tic
    [zsol,~] = fmincon(cost_fcn,zinit,[],[],[],[],[],[],nlcstr_fcn,options);
    %toc
    xfb = zsol(1:3);
    q = zsol(4:end);
    qL = q(1:end/2);
    qR = q(end/2+1:end);
end

