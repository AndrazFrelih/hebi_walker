function [Wlfoot,Wrfoot] = distribute_wrench(Wcom,JL,JR,Hcb,Hbl,Hbr,foot_dims,coul_frict,eq_distr)
    %% constants
    cons_lim = 1;
    % foot size
    X = foot_dims(1)/2*cons_lim;
    Y = foot_dims(2)/2*cons_lim;
    % coulomb friction
    mu = coul_frict*cons_lim;
    % parameters of the contact wrench pyramid (CWP)
    param = [X;Y;mu];
    
    %% compute wrench mappings
    AdT_gcb = Ad_g(Hcb(1:3,1:3),Hcb(1:3,4))';
    AdT_gbl = Ad_g(Hbl(1:3,1:3),Hbl(1:3,4))';
    AdT_gbr = Ad_g(Hbr(1:3,1:3),Hbr(1:3,4))';
    AdT_gcl = AdT_gbl*AdT_gcb;
    AdT_gcr = AdT_gbr*AdT_gcb;
    
    %% define constraints
    % contact wrench pyramid
    U = CWP_exp(param,1);
    % for double support, zmp checking should be adapted
    %U = [U(1:4,:);U(9:end,:)]; 
    % separately consider the z-force (as it should have a threshold)
    pmin = 5;
    efz = [0;0;1;0;0;0];
    
    %% transform optimization quantities
    UL = U*AdT_gcl;
    UR = U*AdT_gcr;
    efzL = efz'*AdT_gcl;
    efzR = efz'*AdT_gcr;
    JLtf = JL'*AdT_gcl;
    JRtf = JR'*AdT_gcr;
    
    %% Formulate optimization problem
    % penalise certain torques
    eps = 1e-6;
    Wjoint = eps*eye(6);
    Wjoint(4,4) = 0.1;
    Wjoint(5,5) = 0.1;
    Wjoint(6,6) = 0.1;
    %Wjoint = zeros(6);
    
    
    % weights
    A1 = 1000;
    A2 = 1;
    % objective function
    J = @(w,wdes) A1 * sq_mat2(wdes-(w(1:end/2)+w(end/2+1:end))) + A2*(sq_mat(JLtf'*w(1:end/2),Wjoint)+sq_mat(JRtf'*w(end/2+1:end),Wjoint));
    wdes = Wcom; %set the desired wrench to the COM one
    Jopt = @(w)J(w,wdes);
    %initial guess for optimization
    w0 = [wdes/2; wdes/2];
    
    % ineq. constraints
    A = [
        UL, zeros(size(UL));
        zeros(size(UR)), UR;
        -efzL, zeros(size(efzL));
        zeros(size(efzR)), -efzR;
    ];
    b = [
        zeros(max(size(UL)),1);
        zeros(max(size(UR)),1);
        -pmin
        -pmin
    ];
    
    options = optimoptions('fmincon','Algorithm','sqp','Display','none');
    [wsol,~] = fmincon(Jopt,w0,A,b,[],[],[],[],[],options);
    % results are computed in the COM frame
    wlleg_com = wsol(1:6);
    wrleg_com = wsol(7:end);
    
    %% foot wrench computation
    if eq_distr
        % distribute the wrench equally
        Wlfoot = AdT_gcl*Wcom /2;
        Wrfoot = AdT_gcr*Wcom /2;
    else
        % distribute wrench according to the optimization result
        Wlfoot = AdT_gcl*wlleg_com;
        Wrfoot = AdT_gcr*wrleg_com;
    end
end

