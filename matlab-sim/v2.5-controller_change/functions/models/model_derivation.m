function model_derivation(robot_data)
    %% define all required states
    syms x y z rollx pitchy yawz real
    syms vx vy vz rollx_p pitchy_p yawz_p wx wy wz real
    syms ax ay az rollx_pp pitchy_pp yawz_pp bx by bz real
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
    
    eul = [rollx pitchy yawz]';
    eul_p = [rollx_p pitchy_p yawz_p]';
    eul_pp = [rollx_pp pitchy_pp yawz_pp]';
    
    X = [r;eul];
    Xp = [v;eul_p];
    Xpp = [a;eul_pp];
    
    qL = [qL1 qL2 qL3 qL4 qL5 qL6]';
    qR = [qR1 qR2 qR3 qR4 qR5 qR6]';
    
    %% derive connection between the chosen convention and general rot. vel
    % the reason for using the xyz notation is the following!! -> since
    % roll and pitch are kept at 0, the mapping B from w=B*ep simplifies to
    % an identity matrix eye(3)
    [B_xyz,Bp_xyz] = eulvel2genvel([eul;eul_p],'xyz');
    [B_zyx,Bp_zyx] = eulvel2genvel([eul;eul_p],'zyx');
    
    % store the two mappings (only xyz convention at the moment)
    path_map = './functions/generated/mappings/';
    create_symbolic_function2("B_XYZ",path_map,B_xyz,eul,[]);
    create_symbolic_function2("Bp_XYZ",path_map,Bp_xyz,[eul;eul_p],[]);
    create_symbolic_function2("B_ZYX",path_map,B_zyx,eul,[]);
    create_symbolic_function2("Bp_ZYX",path_map,Bp_zyx,[eul;eul_p],[]);
    
    %% group transformations -> differences in notation come from different used conventions
    Nlinks = 13;
    robot_obj = cell(Nlinks,1);
    
    % base
    robot_obj{1}.in.H = Transl([x,y,z])*Rot2([rollx,pitchy,yawz]);
    robot_obj{1}.out.N = 3;
    robot_obj{1}.out.H{1} = robot_obj{1}.in.H;
    robot_obj{1}.out.H{2} = Transl(robot_data.mech.left.q1_hip_yaw.pos)*Rot2(robot_data.mech.left.q1_hip_yaw.rot);
    robot_obj{1}.out.H{3} = Transl(robot_data.mech.right.q1_hip_yaw.pos)*Rot2(robot_data.mech.right.q1_hip_yaw.rot);
    robot_obj{1}.com.m = robot_data.mech.base.body.mass;
    robot_obj{1}.com.I = form_Imat(robot_data.mech.base.body.inertia);
    robot_obj{1}.com.H = Transl(robot_data.mech.base.body.com);
    robot_obj{1}.IMU.N = 2;
    robot_obj{1}.IMU.H{1} = Rot2(robot_data.mech.left.q1_hip_yaw.IMU.rot)*robot_obj{1}.out.H{2};
    robot_obj{1}.IMU.H{2} = Rot2(robot_data.mech.right.q1_hip_yaw.IMU.rot)*robot_obj{1}.out.H{3};
    
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
    robot_obj{7}.com.m = robot_data.mech.left.q6_ankle_roll.body.mass;
    robot_obj{7}.com.I = form_Imat(robot_data.mech.left.q6_ankle_roll.body.inertia);
    robot_obj{7}.com.H = Transl(robot_data.mech.left.q6_ankle_roll.body.com);
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
    robot_obj{13}.com.m = robot_data.mech.right.q6_ankle_roll.body.mass;
    robot_obj{13}.com.I = form_Imat(robot_data.mech.right.q6_ankle_roll.body.inertia);
    robot_obj{13}.com.H = Transl(robot_data.mech.right.q6_ankle_roll.body.com);
    robot_obj{13}.IMU.N = 0;
    
    %% compute relative transformations
    lleg_index = [2 7];
    rleg_index = [8 13];
    lleg_dim = lleg_index(2)-lleg_index(1)+1;
    rleg_dim = rleg_index(2)-rleg_index(1)+1;
    
    % memory allocation
    base_in = sym(zeros(4,4));
    base_out = sym(zeros(2,4,4));
    lleg = sym(zeros(lleg_dim,4,4));
    rleg = sym(zeros(rleg_dim,4,4));
    
    base_imu = sym(zeros(2,4,4));
    lleg_imu = sym(zeros(lleg_dim+1,4,4));
    rleg_imu = sym(zeros(rleg_dim+1,4,4));
    
    base_com = sym(zeros(4,4));
    lleg_com = sym(zeros(lleg_dim,4,4));
    rleg_com = sym(zeros(rleg_dim,4,4));
    
    
    %value calculation
    base_in = sym(robot_obj{1}.in.H);
    base_out(1,:,:) = sym(robot_obj{1}.out.H{2});
    base_out(2,:,:) = sym(robot_obj{1}.out.H{3});
    
    base_imu(1,:,:) = sym(robot_obj{1}.IMU.H{1});
    base_imu(2,:,:) = sym(robot_obj{1}.IMU.H{2});
    
    base_com = sym(robot_obj{1}.com.H);
    
    %left leg
    j = 1;
    jimu = 1;
    for i=lleg_index(1):lleg_index(2)
        lleg(j,:,:) = robot_obj{i}.in.H*robot_obj{i}.out.H{1};
        lleg_com(j,:,:) = robot_obj{i}.in.H*robot_obj{i}.com.H;
        j=j+1;
        for k=1:robot_obj{i}.IMU.N
            lleg_imu(jimu,:,:) = robot_obj{i}.IMU.H{k};
            jimu = jimu+1;
        end
    end
    
    %right leg
    j = 1;
    jimu = 1;
    for i=rleg_index(1):rleg_index(2)
        rleg(j,:,:) = robot_obj{i}.in.H*robot_obj{i}.out.H{1};
        rleg_com(j,:,:) = robot_obj{i}.in.H*robot_obj{i}.com.H;
        j=j+1;
        for k=1:robot_obj{i}.IMU.N
            rleg_imu(jimu,:,:) = robot_obj{i}.IMU.H{k};
            jimu = jimu+1;
        end
    end

    %create matlab functions
    Rnd = 4;
    
    path_base = './functions/generated/base/';
    create_symbolic_function2("Hbase_in",path_base,vpa(base_in,Rnd),X,[]);
    create_symbolic_function2("Hbase_out",path_base,vpa(base_out,Rnd),[],[]);
    create_symbolic_function2("Hbase_imu",path_base,vpa(base_imu,Rnd),[],[]);
    create_symbolic_function2("Hbase_com",path_base,vpa(base_com,Rnd),[],[]);
    path_lleg = './functions/generated/lleg/';
    create_symbolic_function2("Hlleg",path_lleg,vpa(lleg,Rnd),qL,[]);
    create_symbolic_function2("Hlleg_imu",path_lleg,vpa(lleg_imu,Rnd),qL,[]);
    create_symbolic_function2("Hlleg_com",path_lleg,vpa(lleg_com,Rnd),qL,[]);
    path_rleg = './functions/generated/rleg/';
    create_symbolic_function2("Hrleg",path_rleg,vpa(rleg,Rnd),qR,[]);
    create_symbolic_function2("Hrleg_imu",path_rleg,vpa(rleg_imu,Rnd),qR,[]);
    create_symbolic_function2("Hrleg_com",path_rleg,vpa(rleg_com,Rnd),qR,[]);
    
    
    % define contact point transforms
    H_foot_fix = zeros(4,4,4);
    H_foot_fix(1,:,:) = Transl([robot_data.mech.foot.corners(1,:),0]'); %back left
    H_foot_fix(2,:,:) = Transl([robot_data.mech.foot.corners(2,:),0]'); %back right
    H_foot_fix(3,:,:) = Transl([robot_data.mech.foot.corners(3,:),0]'); %front left
    H_foot_fix(4,:,:) = Transl([robot_data.mech.foot.corners(4,:),0]'); %front right
    path_eef = "./functions/generated/endeff/";
    create_symbolic_function2("Hcpts",path_eef,sym(H_foot_fix),[],[]);
    
    %create transformations for dampers
    HLknee_fix = (Transl(robot_data.damper.left.knee.pos)*Rot2(robot_data.damper.left.knee.rot)) * Transl([0;0;robot_data.damper.left.knee.offs]);
    HRknee_fix = (Transl(robot_data.damper.right.knee.pos)*Rot2(robot_data.damper.right.knee.rot)) * Transl([0;0;robot_data.damper.right.knee.offs]);
    HLhip_fix = (Transl(robot_data.damper.left.hip.pos)*Rot2(robot_data.damper.left.hip.rot)) * Transl([0;0;robot_data.damper.left.hip.offs]);
    HRhip_fix = (Transl(robot_data.damper.right.hip.pos)*Rot2(robot_data.damper.right.hip.rot)) * Transl([0;0;robot_data.damper.right.hip.offs]);

    %the definition of damper position is only in accordance to the Simscape
    %simulation. Hence some corrections need to be made for the case where
    %simulation and control coordinate frames do not coincide
    HLknee_corr = Transl(-robot_data.mech.left.q4_knee_pitch.pos_2)*Rot2(robot_data.mech.left.q4_knee_pitch.rot_2);
    HRknee_corr = Transl(robot_data.mech.right.q4_knee_pitch.pos_1)*Rot2(robot_data.mech.right.q4_knee_pitch.rot_1);

    HLknee = robot_obj{5}.in.H * HLknee_corr * HLknee_fix;
    HRknee = robot_obj{11}.in.H * HRknee_corr * HRknee_fix;
    HLhip = robot_obj{3}.in.H * HLhip_fix;
    HRhip = robot_obj{9}.in.H * HRhip_fix;

    path_damper = "./functions/generated/damper/";
    create_symbolic_function2("HLdamp_knee",path_damper,vpa(HLknee,Rnd),qL4,[]);
    create_symbolic_function2("HRdamp_knee",path_damper,vpa(HRknee,Rnd),qR4,[]);
    create_symbolic_function2("HLdamp_hip",path_damper,vpa(HLhip,Rnd),qL2,[]);
    create_symbolic_function2("HRdamp_hip",path_damper,vpa(HRhip,Rnd),qR2,[]);