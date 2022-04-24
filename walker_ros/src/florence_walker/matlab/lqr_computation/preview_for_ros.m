function [Pctrl] = preview_for_ros(zdes, Ts, Nprev)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%  Compute the ZMP preview controller according to (Kajita, ICRA 2003)  %%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    

    %% Parameters 
    Pctrl.par.g  = 9.81;     % gravity acceleration
    Pctrl.par.zc = zdes;     % choose a desired COG height
    Pctrl.par.omega = sqrt(Pctrl.par.g/Pctrl.par.zc);
    
    %% the continuous ZMP system
    Pctrl.A = [
        0, 1, 0;
        0, 0, 1;
        0, 0, 0
    ];

    Pctrl.b = [0; 0; 1];
    Pctrl.c = [1, 0, -Pctrl.par.zc/Pctrl.par.g];

    %% the discrete ZMP system
    Pctrl.Ts = Ts;       % sampling time (value from the paper: 5e-3)

    Pctrl.A_d = [
        1, Pctrl.Ts, Ts^2/2;
        0, 1, Ts;
        0, 0, 1
    ];
    Pctrl.b_d = [Ts^3/6; Ts^2/2; Ts];

    %% System augmentation
    Pctrl.N_L = Nprev;         % length of preview: 2 seconds

    Pctrl.Ad = zeros(Pctrl.N_L,Pctrl.N_L);        % Update matrices for xd
    for kk=2:Pctrl.N_L                
        Pctrl.Ad(kk-1,kk) = 1;  
    end                         
    Pctrl.bd = [zeros(Pctrl.N_L-1,1);1];    

    % system for controller design with preview
    Pctrl.AA = [
        1           , Pctrl.c*Pctrl.A_d       , [-1, zeros(1,Pctrl.N_L-1)];
        zeros(3,1)  , Pctrl.A_d         , zeros(3,Pctrl.N_L);
        zeros(Pctrl.N_L,1), zeros(Pctrl.N_L,3), Pctrl.Ad                  
    ];

    Pctrl.BB = [
        Pctrl.c*Pctrl.b_d;
        Pctrl.b_d;
        zeros(Pctrl.N_L,1)
    ];

    Pctrl.CC = [1,zeros(1,3+Pctrl.N_L)];

    %% compute the lqr regulators: values taken from the paper
    Pctrl.Q_e = 1.0;
    Pctrl.Q_x = zeros(3,3);
    Pctrl.R   = 1e-6;

    Pctrl.Q = [  
        Pctrl.Q_e, zeros(1,3), zeros(1,Pctrl.N_L);
        zeros(3,1), Pctrl.Q_x, zeros(3,Pctrl.N_L);
        zeros(Pctrl.N_L,1), zeros(Pctrl.N_L,3), zeros(Pctrl.N_L,Pctrl.N_L)
    ];

    % controller design according to LQR
    Pctrl.K_preview = dlqr(Pctrl.AA, Pctrl.BB, Pctrl.Q, Pctrl.R);

    Pctrl.Gi = Pctrl.K_preview(1);          % K_e
    Pctrl.Gx = Pctrl.K_preview(2:4);        % K_delta
    Pctrl.Gp = Pctrl.K_preview(5:Pctrl.N_L+4);    % K_d

    % plot of the preview gains
    %figure(69);
    %plot(([5:4+Pctrl.N_L]-5)*Ts,-Pctrl.Gp);
    %xlabel('time [s]');
    %ylabel('gain');

end