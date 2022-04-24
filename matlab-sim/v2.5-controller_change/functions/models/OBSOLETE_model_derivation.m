function model_derivation(robot_data)
    %% define all required states
    syms x y z roll pitch yaw real
    syms vx vy vz roll_p pitch_p yaw_p wx wy wz real
    syms ax ay az roll_pp pitch_pp yaw_pp bx by bz real
    syms qL1 qL2 qL3 qL4 qL5 qL6 real
    syms qL1_p qL2_p qL3_p qL4_p qL5_p qL6_p real
    syms qL1_pp qL2_pp qL3_pp qL4_pp qL5_pp qL6_pp real
    syms qR1 qR2 qR3 qR4 qR5 qR6 real
    syms qR1_p qR2_p qR3_p qR4_p qR5_p qR6_p real
    syms qR1_pp qR2_pp qR3_pp qR4_pp qR5_pp qR6_pp real
    
    %% group the states together
    r = [x y z]'; 
    v = [vx vy vz]';
    a = [ax ay az]';
    
    w = [wx wy wz]';
    b = [bx by bz]';
    
    eul = [roll pitch yaw]';
    eul_p = [roll_p pitch_p yaw_p]';
    eul_pp = [roll_pp pitch_pp yaw_pp]';
    
    X = [r;eul];
    Xp = [v;eul_p];
    Xpp = [a;eul_pp];
    
    qL = [qL1 qL2 qL3 qL4 qL5 qL6]';
    qR = [qR1 qR2 qR3 qR4 qR5 qR6]';
    
    %% derive connection between the chosen convention and general rot. vel
    Rwb = RotZ(yaw)*RotY(pitch)*RotX(roll);
    Rwb_p = diff(Rwb,roll)*roll_p + diff(Rwb,pitch)*pitch_p + diff(Rwb,yaw)*yaw_p;
    W_skew = Rwb_p*Rwb';
    wx_d = -W_skew(2,3);
    wy_d = W_skew(1,3);
    wz_d = -W_skew(1,2);
    
    % transformation matrix between euler velocities and rot. velocities: w = B*ep
    B = simplify([
        diff(wx_d,roll_p),diff(wx_d,pitch_p),diff(wx_d,yaw_p);
        diff(wy_d,roll_p),diff(wy_d,pitch_p),diff(wy_d,yaw_p);
        diff(wz_d,roll_p),diff(wz_d,pitch_p),diff(wz_d,yaw_p)
    ]);
    
    % time derivative of the matrix B. Used to get rotational accelerations: b = Bp*ep+B*epp
    Bp = simplify(diff(B,roll)*roll_p + diff(B,pitch)*pitch_p + diff(B,yaw)*yaw_p);
    
    %% group transformations -> differences in notation come from different used conventions
    % base
    robot_obj{1}.in.H = Transl([x,y,z])*Rot2([roll,pitch,yaw]);
    robot_obj{1}.out.N = 3;
    robot_obj{1}.out.H{1} = robot_obj{1}.in.H;
    robot_obj{1}.out.H{2} = Transl(robot_data.mech.left.q1_hip_yaw.pos)*Rot2(robot_data.mech.left.q1_hip_yaw.rot);
    robot_obj{1}.out.H{3} = Transl(robot_data.mech.right.q1_hip_yaw.pos)*Rot2(robot_data.mech.right.q1_hip_yaw.rot);
    robot_obj{1}.com.m = robot_data.mech.base.body.mass;
    robot_obj{1}.com.I = form_Imat(robot_data.mech.base.body.inertia);
    robot_obj{1}.com.H = Transl(robot_data.mech.base.body.com);
    robot_obj{1}.IMU.N = 2;
    robot_obj{1}.IMU.H{1} = Rot2(robot_data.mech.left.q1_hip_yaw.IMU.rot)*robot_obj{1}.out.H{1};
    robot_obj{1}.IMU.H{2} = Rot2(robot_data.mech.right.q1_hip_yaw.IMU.rot)*robot_obj{1}.out.H{2};
    
    % left leg
    robot_obj{2}.in.H = RotZ2(qL1);
    robot_obj{2}.out.N = 1;
    robot_obj{2}.out.H{1} = Transl(robot_data.mech.left.q2_hip_roll.pos)*Rot2(robot_data.mech.left.q2_hip_roll.rot);
    robot_obj{2}.com.m = robot_data.mech.left.q1_hip_yaw.body.mass;
    robot_obj{2}.com.I = form_Imat(robot_data.mech.left.q1_hip_yaw.body.inertia);
    robot_obj{2}.com.H = Transl(robot_data.mech.left.q1_hip_yaw.body.com);
    robot_obj{2}.IMU.N = 1;
    robot_obj{2}.IMU.H{1} = Rot2(robot_data.mech.left.q2_hip_roll.IMU.rot)*robot_obj{2}.out.H{1};
    
    robot_obj{3}.in.H = RotZ2(qL2);
    robot_obj{3}.out.N = 3;
    robot_obj{3}.out.H{1} = Transl(robot_data.mech.left.q3_hip_pitch.pos)*Rot2(robot_data.mech.left.q3_hip_pitch.rot);
    robot_obj{3}.out.H{2} = Transl(robot_data.mech.left.q3_hip_pitch.pos_1)*Rot2(robot_data.mech.left.q3_hip_pitch.rot_1)*robot_obj{3}.out.H{1};
    robot_obj{3}.out.H{3} = Transl(robot_data.mech.left.q3_hip_pitch.pos_2)*Rot2(robot_data.mech.left.q3_hip_pitch.rot_2)*robot_obj{3}.out.H{1};
    robot_obj{3}.com.m = robot_data.mech.left.q2_hip_roll.body.mass;
    robot_obj{3}.com.I = form_Imat(robot_data.mech.left.q2_hip_roll.body.inertia);
    robot_obj{3}.com.H = Transl(robot_data.mech.left.q2_hip_roll.body.com);
    robot_obj{3}.IMU.N = 2;
    robot_obj{3}.IMU.H{1} = Rot2(robot_data.mech.left.q3_hip_pitch.IMU_1.rot)*robot_obj{3}.out.H{2};
    robot_obj{3}.IMU.H{2} = Rot2(robot_data.mech.left.q3_hip_pitch.IMU_2.rot)*robot_obj{3}.out.H{3};
    
    robot_obj{4}.in.H = RotZ2(qL3);
    robot_obj{4}.out.N = 3;
    robot_obj{4}.out.H{1} = Transl(robot_data.mech.left.q4_knee_pitch.pos_virt)*Rot2(robot_data.mech.left.q4_knee_pitch.rot_2);
    robot_obj{4}.out.H{2} = Transl(robot_data.mech.left.q4_knee_pitch.pos_1)*Rot2(robot_data.mech.left.q4_knee_pitch.rot_1)*robot_obj{4}.out.H{1};
    robot_obj{4}.out.H{3} = Transl(robot_data.mech.left.q4_knee_pitch.pos_2)*Rot2(robot_data.mech.left.q4_knee_pitch.rot_2)*robot_obj{4}.out.H{1};
    robot_obj{4}.com.m = robot_data.mech.left.q3_hip_pitch.body.mass;
    robot_obj{4}.com.I = form_Imat(robot_data.mech.left.q3_hip_pitch.body.inertia);
    robot_obj{4}.com.H = Transl(robot_data.mech.left.q3_hip_pitch.body.com);
    robot_obj{4}.IMU.N = 2;
    robot_obj{4}.IMU.H{1} = Rot2(robot_data.mech.left.q4_knee_pitch.IMU_1.rot)*robot_obj{4}.out.H{2};
    robot_obj{4}.IMU.H{2} = Rot2(robot_data.mech.left.q4_knee_pitch.IMU_2.rot)*robot_obj{4}.out.H{3};
    
    robot_obj{5}.in.H = RotZ2(qL4);
    robot_obj{5}.out.N = 1;
    robot_obj{5}.out.H{1} = Transl(robot_data.mech.left.q5_ankle_pitch.pos)*Rot2(robot_data.mech.left.q5_ankle_pitch.rot);
    robot_obj{5}.com.m = robot_data.mech.left.q4_knee_pitch.body.mass;
    robot_obj{5}.com.I = form_Imat(robot_data.mech.left.q4_knee_pitch.body.inertia);
    robot_obj{5}.com.H = Transl(robot_data.mech.left.q4_knee_pitch.body.com);
    robot_obj{5}.IMU.N = 1;
    robot_obj{5}.IMU.H{1} = Rot2(robot_data.mech.left.q5_ankle_pitch.IMU.rot)*robot_obj{5}.out.H{1};
    
    robot_obj{6}.in.H = RotZ2(qL5);
    robot_obj{6}.out.N = 1;
    robot_obj{6}.out.H{1} = Transl(robot_data.mech.left.q6_ankle_roll.pos)*Rot2(robot_data.mech.left.q6_ankle_roll.rot);
    robot_obj{6}.com.m = robot_data.mech.left.q5_ankle_pitch.body.mass;
    robot_obj{6}.com.I = form_Imat(robot_data.mech.left.q5_ankle_pitch.body.inertia);
    robot_obj{6}.com.H = Transl(robot_data.mech.left.q5_ankle_pitch.body.com);
    robot_obj{6}.IMU.N = 1;
    robot_obj{6}.IMU.H{1} = Rot2(robot_data.mech.left.q6_ankle_roll.IMU.rot)*robot_obj{6}.out.H{1};
    
    robot_obj{7}.in.H = RotZ2(qL6);
    robot_obj{7}.out.N = 1;
    robot_obj{7}.out.H{1} = Transl(robot_data.mech.left.ee.pos)*Rot2(robot_data.mech.left.ee.rot);
    robot_obj{7}.com.m = robot_data.mech.left.q5_ankle_pitch.body.mass;
    robot_obj{7}.com.I = form_Imat(robot_data.mech.left.q5_ankle_pitch.body.inertia);
    robot_obj{7}.com.H = Transl(robot_data.mech.left.q5_ankle_pitch.body.com);
    robot_obj{7}.IMU.N = 0;
    
    %right
    robot_obj{8}.in.H = RotZ2(qR1);
    robot_obj{8}.out.N = 1;
    robot_obj{8}.out.H{1} = Transl(robot_data.mech.right.q2_hip_roll.pos)*Rot2(robot_data.mech.right.q2_hip_roll.rot);
    robot_obj{8}.com.m = robot_data.mech.right.q1_hip_yaw.body.mass;
    robot_obj{8}.com.I = form_Imat(robot_data.mech.right.q1_hip_yaw.body.inertia);
    robot_obj{8}.com.H = Transl(robot_data.mech.right.q1_hip_yaw.body.com);
    robot_obj{8}.IMU.N = 1;
    robot_obj{8}.IMU.H{1} = Rot2(robot_data.mech.right.q2_hip_roll.IMU.rot)*robot_obj{8}.out.H{1};
    
    robot_obj{9}.in.H = RotZ2(qR2);
    robot_obj{9}.out.N = 3;
    robot_obj{9}.out.H{1} = Transl(robot_data.mech.right.q3_hip_pitch.pos)*Rot2(robot_data.mech.right.q3_hip_pitch.rot);
    robot_obj{9}.out.H{2} = Transl(robot_data.mech.right.q3_hip_pitch.pos_1)*Rot2(robot_data.mech.right.q3_hip_pitch.rot_1)*robot_obj{9}.out.H{1};
    robot_obj{9}.out.H{3} = Transl(robot_data.mech.right.q3_hip_pitch.pos_2)*Rot2(robot_data.mech.right.q3_hip_pitch.rot_2)*robot_obj{9}.out.H{1};
    robot_obj{9}.com.m = robot_data.mech.right.q2_hip_roll.body.mass;
    robot_obj{9}.com.I = form_Imat(robot_data.mech.right.q2_hip_roll.body.inertia);
    robot_obj{9}.com.H = Transl(robot_data.mech.right.q2_hip_roll.body.com);
    robot_obj{9}.IMU.N = 2;
    robot_obj{9}.IMU.H{1} = Rot2(robot_data.mech.right.q3_hip_pitch.IMU_1.rot)*robot_obj{9}.out.H{2};
    robot_obj{9}.IMU.H{2} = Rot2(robot_data.mech.right.q3_hip_pitch.IMU_2.rot)*robot_obj{9}.out.H{3};
    
    
    robot_obj{10}.in.H = RotZ2(qR3);
    robot_obj{10}.out.N = 3;
    robot_obj{10}.out.H{1} = Transl(robot_data.mech.right.q4_knee_pitch.pos_virt)*Rot2(robot_data.mech.right.q4_knee_pitch.rot_2);
    robot_obj{10}.out.H{2} = Transl(robot_data.mech.right.q4_knee_pitch.pos_1)*Rot2(robot_data.mech.right.q4_knee_pitch.rot_1)*robot_obj{10}.out.H{1};
    robot_obj{10}.out.H{3} = Transl(robot_data.mech.right.q4_knee_pitch.pos_2)*Rot2(robot_data.mech.right.q4_knee_pitch.rot_2)*robot_obj{10}.out.H{1};
    robot_obj{10}.com.m = robot_data.mech.right.q3_hip_pitch.body.mass;
    robot_obj{10}.com.I = form_Imat(robot_data.mech.right.q3_hip_pitch.body.inertia);
    robot_obj{10}.com.H = Transl(robot_data.mech.right.q3_hip_pitch.body.com);
    robot_obj{10}.IMU.N = 2;
    robot_obj{10}.IMU.H{1} = Rot2(robot_data.mech.right.q4_knee_pitch.IMU_1.rot)*robot_obj{10}.out.H{2};
    robot_obj{10}.IMU.H{2} = Rot2(robot_data.mech.right.q4_knee_pitch.IMU_2.rot)*robot_obj{10}.out.H{3};
    
    robot_obj{11}.in.H = RotZ2(qR4);
    robot_obj{11}.out.N = 1;
    robot_obj{11}.out.H{1} = Transl(robot_data.mech.right.q5_ankle_pitch.pos)*Rot2(robot_data.mech.right.q5_ankle_pitch.rot);
    robot_obj{11}.com.m = robot_data.mech.right.q4_knee_pitch.body.mass;
    robot_obj{11}.com.I = form_Imat(robot_data.mech.right.q4_knee_pitch.body.inertia);
    robot_obj{11}.com.H = Transl(robot_data.mech.right.q4_knee_pitch.body.com);
    robot_obj{11}.IMU.N = 1;
    robot_obj{11}.IMU.H{1} = Rot2(robot_data.mech.right.q5_ankle_pitch.IMU.rot)*robot_obj{11}.out.H{1};
    
    robot_obj{12}.in.H = RotZ2(qR5);
    robot_obj{12}.out.N = 1;
    robot_obj{12}.out.H{1} = Transl(robot_data.mech.right.q6_ankle_roll.pos)*Rot2(robot_data.mech.right.q6_ankle_roll.rot);
    robot_obj{12}.com.m = robot_data.mech.right.q5_ankle_pitch.body.mass;
    robot_obj{12}.com.I = form_Imat(robot_data.mech.right.q5_ankle_pitch.body.inertia);
    robot_obj{12}.com.H = Transl(robot_data.mech.right.q5_ankle_pitch.body.com);
    robot_obj{12}.IMU.N = 1;
    robot_obj{12}.IMU.H{1} = Rot2(robot_data.mech.right.q6_ankle_roll.IMU.rot)*robot_obj{12}.out.H{1};
    
    robot_obj{13}.in.H = RotZ2(qR6);
    robot_obj{13}.out.N = 1;
    robot_obj{13}.out.H{1} = Transl(robot_data.mech.right.ee.pos)*Rot2(robot_data.mech.right.ee.rot);
    robot_obj{13}.com.m = robot_data.mech.right.q5_ankle_pitch.body.mass;
    robot_obj{13}.com.I = form_Imat(robot_data.mech.right.q5_ankle_pitch.body.inertia);
    robot_obj{13}.com.H = Transl(robot_data.mech.right.q5_ankle_pitch.body.com);
    robot_obj{13}.IMU.N = 0;
    
    %% compute absolute transformations
    % set up the rules according to which the structure is to be traversed
    Ntra = max(size(robot_obj));
    % abs. transf.
    abs_tf_exp = ones(Ntra,1);
    abs_tf_exp(1) = robot_obj{1}.out.N;
    abs_tf_names = ["Hwb","Hw0L","Hw0R","Hw1L","Hw2L","Hw3L","Hw4L","Hw5L","HweeL","Hw1R","Hw2R","Hw3R","Hw4R","Hw5R","HweeR"];
    abs_tf_Hpr =   ["",  "Hwb", "Hwb", "Hw0L","Hw1L","Hw2L","Hw3L","Hw4L","Hw5L","Hw0R","Hw1R","Hw2R","Hw3R","Hw4R","Hw5R"];
    abs_tf_ord =   [1,   2,     9,     3,     4,     5,     6,      7,     8,     10,    11,    12,    13,    14,     15];
    abs_tf_out = cell(max(size(abs_tf_names)),1);
    
    abs_tf_inp_names = ["Hwb_in","Hw1L_in","Hw2L_in","Hw3L_in","Hw4L_in","Hw5L_in","HweeL_in","Hw1R_in","Hw2R_in","Hw3R_in","Hw4R_in","Hw5R_in","HweeR_in"];
    abs_tf_inp = cell(max(size(abs_tf_inp_names)),1);
    
    % IMU transf.
    imu_tf_names = ["Hw0L_imu","Hw0R_imu","Hw1L_imu","Hw2_1L_imu","Hw2_2L_imu","Hw3_1L_imu","Hw3_2L_imu","Hw4L_imu","Hw5L_imu","Hw1R_imu","Hw2_1R_imu","Hw2_2R_imu","Hw3_1R_imu","Hw3_2R_imu","Hw4R_imu","Hw5R_imu"];
    imu_tf_Hpr =   ["Hwb",    "Hwb",    "Hw1L",   "Hw2L",     "Hw2L",     "Hw3L",     "Hw3L",     "Hw4L",   "Hw5L",   "Hw1R",   "Hw2R",     "Hw2R",     "Hw3R",     "Hw3R",     "Hw4R",   "Hw5R"];
    imu_tf_ord =   [1,       9,       2,       3,         4,         5,         6,         7,       8,       10,      11,        12,        13,        14,        15,      16];
    imu_tf = cell(max(size(imu_tf_names)),1);
    % COM transf. (todo)
    com_tf_names = ["Hwb_com","Hw1L_com","Hw2L_com","Hw3L_com","Hw4L_com","Hw5L_com","HweeL_com","Hw1R_com","Hw2R_com","Hw3R_com","Hw4R_com","Hw5R_com","HweeR_com"];
    com_tf = cell(max(size(com_tf_names)),1);
    
    cnt_abs = 1;
    cnt_imu = 1;
    for i=1:Ntra
        %check how many output frames a link has
        Nout = abs_tf_exp(i);
        for j=1:Nout
            %give the tf a correct name and put it in the right place
            abs_tf_out{abs_tf_ord(cnt_abs)}.name = abs_tf_names(cnt_abs);
            
            %find an index at which the previous absolute Tf is located
            ind = find(abs_tf_names == abs_tf_Hpr(cnt_abs));
            %get that previous absolute transformation
            if isempty(ind)
                Hprev = eye(4);
            else
                Hprev = abs_tf_out{abs_tf_ord(ind)}.H;
            end
            %compute the absolute Tf for this frame and store it in the
            %right place
            abs_tf_out{abs_tf_ord(cnt_abs)}.H = Hprev * robot_obj{i}.in.H * robot_obj{i}.out.H{j};
            cnt_abs = cnt_abs + 1;
            
            if j==1
                abs_tf_inp{i}.H = Hprev * robot_obj{i}.in.H;
                abs_tf_inp{i}.name = abs_tf_inp_names(i);
            end
        end
        
    end
    
    for i=1:Ntra
        %check how many IMU frames a link has
        Nimu = robot_obj{i}.IMU.N;
        for j=1:Nimu
            %find the transformation frame, to which the IMU is rigidly
            %attached to
            ind = find(abs_tf_names == imu_tf_Hpr(cnt_imu));
            Hprev = abs_tf_out{abs_tf_ord(ind)}.H;
            %name the IMU frame correctly and put it in the right place
            imu_tf{imu_tf_ord(cnt_imu)}.name = imu_tf_names(cnt_imu);
            %compute the absolute Tf and put it in the right place
            imu_tf{imu_tf_ord(cnt_imu)}.H = Hprev * robot_obj{i}.IMU.H{j}; 
            cnt_imu = cnt_imu + 1;
        end
    end
    
    for i=1:Ntra
        com_tf{i}.H = abs_tf_inp{i}.H*robot_obj{i}.com.H;
        com_tf{i}.name = com_tf_names(i);
    end
    
    % fix the base to get faster local calculations
    % joint absolute transformations
    abs_tf_base = abs_tf_out;
    N1 = max(size(abs_tf_base));
    for i=1:N1
        abs_tf_base{i}.name = replace(abs_tf_base{i}.name,'w','b');
        abs_tf_base{i}.H = subs(abs_tf_base{i}.H, X, zeros(6,1));
    end
    
    % imu absolute transformations
    imu_tf_base = imu_tf;
    N2 = max(size(imu_tf_base));
    for i=1:N2
        imu_tf_base{i}.name = replace(imu_tf_base{i}.name,'w','b');
        imu_tf_base{i}.H = subs(imu_tf_base{i}.H, X, zeros(6,1));
    end
    
    % com absolute transformations
    com_tf_base = com_tf;
    N3 = max(size(com_tf_base));
    for i=1:N3
        com_tf_base{i}.name = replace(com_tf{i}.name,'w','b');
        com_tf_base{i}.H = subs(com_tf{i}.H, X, zeros(6,1));
    end
    
    %% store absolute transformations
    redo_transforms = 0; 
    if redo_transforms
        path_Tf = "./functions/generated/transforms/";
        var_list_w = [X;qL;qR];
        var_list_b = [qL;qR];
        for i=1:N1
            create_symbolic_function(abs_tf_out{i}.name,path_Tf,abs_tf_out{i}.H,var_list_w,[]);
            create_symbolic_function(abs_tf_base{i}.name,path_Tf,abs_tf_base{i}.H,var_list_b,[]);
        end
        
        for i=1:N2
            create_symbolic_function(imu_tf{i}.name,path_Tf,imu_tf{i}.H,var_list_w,[]);
            create_symbolic_function(imu_tf_base{i}.name,path_Tf,imu_tf_base{i}.H,var_list_b,[]);
        end
        
        for i=1:N3
            create_symbolic_function(com_tf{i}.name,path_Tf,com_tf{i}.H,var_list_w,[]);
            create_symbolic_function(com_tf_base{i}.name,path_Tf,com_tf_base{i}.H,var_list_b,[]);
        end
    end
    
    %% compute geometric jacobians
    % floating base velocities get directly mapped to spatial velocities
    Jb = eye(6);
    
    %generate jacobians for all the links (generate separate jacobians for
    %both legs, as they are kinematically independent)
    lleg_names = ["Hw0L","Hw1L","Hw2L","Hw3L","Hw4L","Hw5L","HweeL"];
    rleg_names = ["Hw0R","Hw1R","Hw2R","Hw3R","Hw4R","Hw5R","HweeR"];
    
    leg_names{1} = lleg_names;
    leg_names{2} = rleg_names;
    
    Ndof_leg = 6;
    Ndof = 6;
    
    Jac = cell(2,1);
    Jac{1} = cell(Ndof_leg,1);
    Jac{2} = cell(Ndof_leg,1);
    
    for k=1:2
        %create jacobians for all links starting from 1 onwards
        for i=1:Ndof_leg
            iend = abs_tf_ord(abs_tf_names == leg_names{k}(i));
            Hend = abs_tf_out{iend}.H;
            tend = reshape(Hend(1:3,4),3,1);
            Ji = sym(zeros(Ndof,Ndof_leg));
            for j=1:Ndof_leg
                if(j<=i)
                    if j==1
                        Hstart = eye(4);
                    else
                        istart = abs_tf_ord(abs_tf_names == leg_names{k}(i-1));
                        Hstart = abs_tf_out{istart}.H;
                    end
                    tstart = reshape(Hstart(1:3,4),3,1);
                    Zcf = reshape(Hstart(1:3,3),3,1);

                    dT = tend - tstart;
                    Jvij = cross(Zcf,dT);
                    Jwij = Zcf;
                    Jij = [Jvij;Jwij];
                else
                    %later links in the chain, do not contribute to the velocity of this
                    %link (kinematically speaking)
                    Jij = sym(zeros(6,1)); 
                end
                Ji(:,j) = Jij;
            end
            name = "Jw"+i;
            if(k==1)
                name = name+"L";
            else
                name = name+"R";
            end
            Jac{k}{i}.name = name;
            Jac{k}{i}.J = Ji;
        end
    end
    
    %create jacobians that are only valid relative to the base
    Jac_base = Jac;
    for i=1:Ndof_leg
        Jac_base{1}{i}.name = replace(Jac_base{1}{i}.name,'w','b');
        Jac_base{1}{i}.J = subs(Jac_base{1}{i}.J, X, zeros(6,1));
    end
    
    %% store geometric jacobians
    %only redo functions if set to nonzero
    redo_jacobians = 1; 
    if redo_jacobians
        path_J = "./functions/generated/jacobians/";
        var_list_w{1} = [X;qL];
        var_list_w{2} = [X;qR];
        var_list_b{1} = qL;
        var_list_b{2} = qR;
        for k=1:2
            for i=1:Ndof_leg
                create_symbolic_function(Jac{k}{i}.name,path_J,Jac{k}{i}.J,var_list_w{k},[]);
                create_symbolic_function(Jac_base{k}{i}.name,path_J,Jac_base{k}{i}.J,var_list_b{k},[]);
            end
        end
    end
    %there is a better more efficient way of computing jacobians
    
end

