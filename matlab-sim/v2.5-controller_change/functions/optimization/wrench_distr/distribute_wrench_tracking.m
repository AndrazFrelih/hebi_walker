function [Wlfoot,Wrfoot] = distribute_wrench_tracking(Wcom,JL,JR,Hcb,Hbl,Hbr,foot_dims,coul_frict,eq_distr,phase,WshiftL,WshiftR)
    %% variables for the hot start of the optimization
    persistent init;
    if isempty(init)
        init = 1;
    end
    
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
    % separately consider the z-force (as it should have a threshold)
    pmin = 15;
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
    eps = 1e-12;
    Wjoint = eps*eye(6);
    Wjoint(5,5) = Wjoint(5,5)*100;
    Wjoint(6,6) = Wjoint(6,6)*100;
    Wjoint = zeros(6);
    
    options = optimoptions('fmincon','Algorithm','sqp','Display','none');
    if phase == 0 %double support
        %% weights
        A1 = 10000;
        A2 = 1;
        A3 = 0.1;
        %% objective function formulation
        %initial guess for optimization
        w0_ds = [Wcom /2; Wcom /2];
        % desired value
        wdes = Wcom;
        % best possible projection of the com wrench
        J1 = @(w) A1 * sq_mat2(wdes-(w(1:end/2)+w(end/2+1:end)));
        % limit torques in certain joints
        J2 = @(w) A2 * (sq_mat(JLtf'*w(1:end/2),Wjoint)+sq_mat(JRtf'*w(end/2+1:end),Wjoint));
        % smooth wrench transition between legs when shifting weight
        Wad = 0.5;
        WshLada = WshiftL+Wad;
        WshRada = WshiftR+Wad;
        WshTot = WshLada+WshRada;
        WshLada = WshLada/WshTot;
        WshRada = WshRada/WshTot;
        
        J3 = @(w) A3 * sq_mat2((1-WshLada)*efzL*w(1:end/2) - (1-WshRada)*efzR*w(end/2+1:end));
        % all the costs combined
        Jopt_ds = @(w)J1(w)+J2(w)+J3(w);
        

        % ineq. constraints
        A_ds = [
            UL, zeros(size(UL));
            zeros(size(UR)), UR;
            efzL, zeros(size(efzL));
            zeros(size(efzR)), efzR;
        ];
        b_ds = [
            zeros(max(size(UL)),1);
            zeros(max(size(UR)),1);
            -pmin
            -pmin
        ];

        [wsol_ds,~] = fmincon(Jopt_ds,w0_ds,A_ds,b_ds,[],[],[],[],[],options);
        % results are computed in the COM frame
        wlleg_com = wsol_ds(1:6);
        wrleg_com = wsol_ds(7:end);
        
    else %single support
        %% weights
        A1 = 10000;
        %% objective function formulation
        wdes = Wcom;
        % best possible projection of the com wrench
        J1 = @(w) A1 * sq_mat2(wdes-w);
        % all the costs combined
        Jopt_ss = @(w)J1(w);
        %initial guess for optimization
        w0_ss = Wcom;
            
        
        if WshiftL>WshiftR
            % ineq. constraints
            A_ss = [
                UL;
                efzL;
            ];
            b_ss = [
                zeros(size(UL,1),1);
                -pmin
            ];
        else
            % ineq. constraints
            A_ss = [
                UR;
                efzR;
            ];
            b_ss = [
                zeros(size(UR,1),1);
                -pmin
            ];
        end
        
        [wsol_ss,~] = fmincon(Jopt_ss,w0_ss,A_ss,b_ss,[],[],[],[],[],options);
        
        % determine, which leg should output the torque
        if WshiftL>WshiftR
            wlleg_com = wsol_ss;
            wrleg_com = zeros(6,1);
        else
            wlleg_com = zeros(6,1);
            wrleg_com = wsol_ss;
        end
    end
    
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

