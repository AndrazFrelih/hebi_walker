%% VERSION 2.3.

function [collision] = get_collision_geom()
    unit = 1e-3;
    %bounding box (for collisions)
        X8.L = 0.111;
        X8.W = 0.074;
        X8.H = 0.044;
        X5.L = X8.L;
        X5.W = X8.W;
        X5.H = 0.030;
    
    %% Base collisions
    base.N = 3;
    %base
        col.type = "cyl";
        enl = 1.05;
        col.dim = [0.301*enl,0.038*enl]; %rad & height
        col.pos = zeros(1,3);
        col.rot = zeros(1,3);
        base.col{1} = col;
    %first motor
        col.type = "box";
        col.dim = [X8.L,X8.W,X8.H];
        col.pos = [(197.573+138.445)/2,(37.207+156.793)/2,(-65.00-21.00)/2]*unit;
        col.rot = [0,0,+pi/6];
        base.col{2} = col;
    %second motor
        col.pos = [(197.573+138.445)/2,(-37.207-156.793)/2,(-65.00-21.00)/2]*unit;
        col.rot = [0,0,-pi/6];
        base.col{3} = col;
        
    %% Link 1
    EL = 1;
    col.type = "box";
    L1_holder_dim = [15.5,40,105]*unit;
    L1_motor_dim = [X8.H,X8.L,X8.W];
    %left side
    lleg{EL}.N = 2;
        %holder
        col.dim = L1_holder_dim*unit;
        col.pos = [-38.00,-35.00,45.05]*unit+col.dim/2;
        col.rot = zeros(1,3);
        lleg{EL}.col{1} = col;
        %motor
        col.dim = L1_motor_dim;
        col.pos = [-81.50,-37.00,63.05]*unit+col.dim/2;
        col.rot = zeros(1,3);
        lleg{EL}.col{2} = col;
        
    %right side
    rleg{EL}.N = lleg{EL}.N;
        %holder
        col.dim = L1_holder_dim*unit;
        col.pos = [-38.00,-25.00,45.05]*unit+col.dim/2;
        col.rot = zeros(1,3);
        rleg{EL}.col{1} = col;
        %motor
        col.dim = L1_motor_dim;
        col.pos = [-81.50,-74.00,63.05]*unit+col.dim/2;
        col.rot = zeros(1,3);
        rleg{EL}.col{2} = col;
        
    %% Link 2
    EL = 2;
    col.type = "box";
    L2_motor_dim = [X8.L,X8.H,X8.W];
    %left side
    lleg{EL}.N = 3;
        %motor 1
            col.dim = L2_motor_dim;
            col.pos = [-37.0,-81.5,63.05]*unit+col.dim/2;
            col.rot = zeros(1,3);
            lleg{EL}.col{1} = col;
        %motor 2
            col.dim = L2_motor_dim;
            col.pos = [-37.0,37.0,63.05]*unit+col.dim/2;
            col.rot = zeros(1,3);
            lleg{EL}.col{2} = col;
        %holder
            col.dim = [60,75,105]*unit;
            col.pos = [-35.0,-37.5,45.05]*unit+col.dim/2;
            col.rot = zeros(1,3);
            lleg{EL}.col{3} = col;
    %right side (identic parts)
    rleg{EL} = lleg{EL};
    
    %% Link 3
    EL = 3;
    col.type = "box";
    L3_motor_dim = [X8.L,X8.W,X8.H];
    L3_rod = [300,48,35]*unit;
    %left side
    lleg{EL}.N = 4;
        %motor 1
            col.dim = L3_motor_dim;
            col.pos = [251.50,-37.00,-82.55]*unit+col.dim/2;
            col.rot = zeros(1,3);
            lleg{EL}.col{1} = col;
        %motor 2
            col.dim = L3_motor_dim;
            col.pos = [251.50,-37.00,38.55]*unit+col.dim/2;
            col.rot = zeros(1,3);
            lleg{EL}.col{2} = col;
        %rod 1
            col.dim = L3_rod;
            col.pos = [12.50,-24.00,-120.05]*unit+col.dim/2;
            col.rot = zeros(1,3);
            lleg{EL}.col{3} = col;
        %rod 2
            col.dim = L3_rod;
            col.pos = [12.50,-24.00,85.05]*unit+col.dim/2;
            col.rot = zeros(1,3);
            lleg{EL}.col{4} = col;
    %right side (identic parts)
    rleg{EL} = lleg{EL};        
    
    %% Link 4
    EL = 4;
    col.type = "box";
    L4_motor_dim = L3_motor_dim;
    L4_motor_pos = [251.00,-37.00,-64.00]*unit+L4_motor_dim/2;
    %left side
    corr_rot = eul2rotm([0,0,pi]); %has to be rotated around x prior to translation (hence col.rot is still zero)
    lleg{EL}.N = 3;
        %motor
            col.dim = L4_motor_dim;
            col.pos = (corr_rot*L4_motor_pos')';
            col.rot = zeros(1,3);
            lleg{EL}.col{1} = col;
        %rod
            col.dim = [322.50,48.00,37.00]*unit;
            col.pos = [35.50,-24.00,-19.50]*unit+col.dim/2;
            col.rot = zeros(1,3);
            lleg{EL}.col{2} = col;
        %holder
            col.dim = [60.00,48.00,74.90]*unit;
            col.pos = [-24.50,-24.00,-37.45]*unit+col.dim/2;
            col.rot = zeros(1,3);
            lleg{EL}.col{3} = col;
    %right side (identic parts)
    rleg{EL} = lleg{EL};
        %motor (minor correction)
            rleg{EL}.col{1}.pos = L4_motor_pos;
    
    %% Link 5
    EL = 5;
    col.type = "box";
    L5_motor_dim = [X8.L,X8.H,X8.W];
    L5_holder = [60.00,15.50,105.00]*unit;
    %left side
    lleg{EL}.N = 2;
        %motor
            col.dim = L5_motor_dim;
            col.pos = [-74.00,-21.50,62.05]*unit+col.dim/2;
            col.rot = zeros(1,3);
            lleg{EL}.col{1} = col;
        %holder
            col.dim = L5_holder;
            col.pos = [-25.00,22.00,44.05]*unit+L5_holder/2;
            col.rot = zeros(1,3);
            lleg{EL}.col{2} = col;
        
    %right side
    rleg{EL}.N = lleg{EL}.N;
        %motor
            col.dim = L5_motor_dim;
            col.pos = [-74.00,-23.00,62.05]*unit+col.dim/2;
            col.rot = zeros(1,3);
            rleg{EL}.col{1} = col;
        %holder
            col.dim = L5_holder;
            col.pos = [-25.00,-37.50,44.05]*unit+L5_holder/2;
            col.rot = zeros(1,3);
            rleg{EL}.col{2} = col;
            
    %% Link 6
    EL = 6;
    col.type = "box";
    %left side
    lleg{EL}.N = 1;
        %foot
            col.dim = [36.675,149.00,167.00]*unit;
            col.pos = [82.50,-74.50,-50.50]*unit+col.dim/2;
            col.rot = zeros(1,3);
            lleg{EL}.col{1} = col;
    %right side (identic parts)
    rleg{EL} = lleg{EL};
    
    %% Return the values
    collision.base = base;
    collision.lleg = lleg;
    collision.rleg = rleg;
end

