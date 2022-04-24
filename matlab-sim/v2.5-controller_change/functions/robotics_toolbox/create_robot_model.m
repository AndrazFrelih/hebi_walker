%% VERSION 2.3.

function [robot_model,robot_model_dyn,frame_list] = create_robot_model(rob_mech, visualise)
    %this is going to be a fully kinematic model (for inverse kinematics
    %purposes)
    
    %% create a robot
    robot_model = rigidBodyTree;  
    
    %% KINEMATIC MODEL
    
    %% BASE
    % define the robot base
    link_list.base = "FloBase";
    lnk_base = rigidBody(link_list.base);
    %define the fixed joint
    joint_list.base = "FloBaseJoint";
    jnt_base = rigidBodyJoint(joint_list.base,"fixed");
    %connect the fixed joint to the robot base
    lnk_base.Joint = jnt_base;
    %add visuals to the body
    addVisual(lnk_base,"Mesh","Base_rem.stl");   
    
    %base com frame
    com_list.base = "BaseCom";
    com_base = rigidBody(com_list.base);
    com_base_Tf = trvec2tform(rob_mech.base.body_rt.com);
    setFixedTransform(com_base.Joint,com_base_Tf);
    
    %% set the euler convention
    eul_ord = "xyz";
    
    %% LINK 1
    % define the first link of the left leg
    link_list.lleg(1) = "Link1L";
    lnk_1L = rigidBody(link_list.lleg(1));
    %define the rotational joint and set home position
    joint_list.lleg(1) = "LefHipYaw";
    jnt_1L = rigidBodyJoint(joint_list.lleg(1),"revolute");
    jnt_1L.HomePosition = rob_mech.left.q1_hip_yaw.qinit;
    %define the relative transformation
    jnt_1L_Tf = trvec2tform(rob_mech.left.q1_hip_yaw.pos)*eul2tform(rob_mech.left.q1_hip_yaw.rot,eul_ord);
    setFixedTransform(jnt_1L,jnt_1L_Tf);
    %connect the joint to the link
    lnk_1L.Joint = jnt_1L;
    %add visuals to the body
    addVisual(lnk_1L,"Mesh","Link1L_rem.stl");  
    
    %left link 1 com frame
    com_list.lleg(1) = "Link1LCom";
    com_lnk_1L = rigidBody(com_list.lleg(1));
    com_lnk_1L_Tf = trvec2tform(rob_mech.left.q1_hip_yaw.body_rt.com);
    setFixedTransform(com_lnk_1L.Joint,com_lnk_1L_Tf);
    
    
    % define the first link of the right leg
    link_list.rleg(1) = "Link1R";
    lnk_1R = rigidBody(link_list.rleg(1));
    %define the rotational joint and set home position
    joint_list.rleg(1) = "RigHipYaw";
    jnt_1R = rigidBodyJoint(joint_list.rleg(1),"revolute");
    jnt_1R.HomePosition = rob_mech.right.q1_hip_yaw.qinit;
    %define the relative transformation
    jnt_1R_Tf = trvec2tform(rob_mech.right.q1_hip_yaw.pos)*eul2tform(rob_mech.right.q1_hip_yaw.rot,eul_ord);
    setFixedTransform(jnt_1R,jnt_1R_Tf);
    %connect the joint to the link
    lnk_1R.Joint = jnt_1R;
    %add visuals to the body
    addVisual(lnk_1R,"Mesh","Link1R_rem.stl");  
    
    %right link 1 com frame
    com_list.rleg(1) = "Link1RCom";
    com_lnk_1R = rigidBody(com_list.rleg(1));
    com_lnk_1R_Tf = trvec2tform(rob_mech.right.q1_hip_yaw.body_rt.com);
    setFixedTransform(com_lnk_1R.Joint,com_lnk_1R_Tf);
    
    
    %% LINK 2
    % define the first link of the left leg
    link_list.lleg(2) = "Link2L";
    lnk_2L = rigidBody(link_list.lleg(2));
    %define the rotational joint and set home position
    joint_list.lleg(2) = "LefHipRoll";
    jnt_2L = rigidBodyJoint(joint_list.lleg(2),"revolute");
    jnt_2L.HomePosition = rob_mech.left.q2_hip_roll.qinit;
    %define the relative transformation
    jnt_2L_Tf = trvec2tform(rob_mech.left.q2_hip_roll.pos)*eul2tform(rob_mech.left.q2_hip_roll.rot,eul_ord);
    setFixedTransform(jnt_2L,jnt_2L_Tf);
    %connect the joint to the link
    lnk_2L.Joint = jnt_2L;
    %add visuals to the body
    addVisual(lnk_2L,"Mesh","Link2_rem.stl");  
    
    %left link 2 com frame
    com_list.lleg(2) = "Link2LCom";
    com_lnk_2L = rigidBody(com_list.lleg(2));
    com_lnk_2L_Tf = trvec2tform(rob_mech.left.q2_hip_roll.body_rt.com);
    setFixedTransform(com_lnk_2L.Joint,com_lnk_2L_Tf);
    
    
    % define the first link of the right leg
    link_list.rleg(2) = "Link2R";
    lnk_2R = rigidBody(link_list.rleg(2));
    %define the rotational joint and set home position
    joint_list.rleg(2) = "RigHipRoll";
    jnt_2R = rigidBodyJoint(joint_list.rleg(2),"revolute");
    jnt_2R.HomePosition = rob_mech.right.q2_hip_roll.qinit;
    %define the relative transformation
    jnt_2R_Tf = trvec2tform(rob_mech.right.q2_hip_roll.pos)*eul2tform(rob_mech.right.q2_hip_roll.rot,eul_ord);
    setFixedTransform(jnt_2R,jnt_2R_Tf);
    %connect the joint to the link
    lnk_2R.Joint = jnt_2R;
    %add visuals to the body
    addVisual(lnk_2R,"Mesh","Link2_rem.stl");  
    
    %right link 2 com frame
    com_list.rleg(2) = "Link2RCom";
    com_lnk_2R = rigidBody(com_list.rleg(2));
    com_lnk_2R_Tf = trvec2tform(rob_mech.right.q2_hip_roll.body_rt.com);
    setFixedTransform(com_lnk_2R.Joint,com_lnk_2R_Tf);
    
    
    %% LINK 3
    % define the first link of the left leg
    link_list.lleg(3) = "Link3L";
    lnk_3L = rigidBody(link_list.lleg(3));
    %define the rotational joint and set home position
    joint_list.lleg(3) = "LefHipPitch";
    jnt_3L = rigidBodyJoint(joint_list.lleg(3),"revolute");
    jnt_3L.HomePosition = rob_mech.left.q3_hip_pitch.qinit;
    %define the relative transformation
    jnt_3L_Tf = trvec2tform(rob_mech.left.q3_hip_pitch.pos)*eul2tform(rob_mech.left.q3_hip_pitch.rot,eul_ord);
    setFixedTransform(jnt_3L,jnt_3L_Tf);
    %connect the joint to the link
    lnk_3L.Joint = jnt_3L;
    %add visuals to the body
    addVisual(lnk_3L,"Mesh","Link3_rtbox_rem.stl");  
    
    %left link 3 com frame
    com_list.lleg(3) = "Link3LCom";
    com_lnk_3L = rigidBody(com_list.lleg(3));
    com_lnk_3L_Tf = trvec2tform(rob_mech.left.q3_hip_pitch.body_rt.com);
    setFixedTransform(com_lnk_3L.Joint,com_lnk_3L_Tf);
    
    
    % define the first link of the right leg
    link_list.rleg(3) = "Link3R";
    lnk_3R = rigidBody(link_list.rleg(3));
    %define the rotational joint and set home position
    joint_list.rleg(3) = "RigHipPitch";
    jnt_3R = rigidBodyJoint(joint_list.rleg(3),"revolute");
    jnt_3R.HomePosition = rob_mech.right.q3_hip_pitch.qinit;
    %define the relative transformation
    jnt_3R_Tf = trvec2tform(rob_mech.right.q3_hip_pitch.pos)*eul2tform(rob_mech.right.q3_hip_pitch.rot,eul_ord);
    setFixedTransform(jnt_3R,jnt_3R_Tf);
    %connect the joint to the link
    lnk_3R.Joint = jnt_3R;
    %add visuals to the body
    addVisual(lnk_3R,"Mesh","Link3_rtbox_rem.stl");  
    
    %right link 3 com frame
    com_list.rleg(3) = "Link3RCom";
    com_lnk_3R = rigidBody(com_list.rleg(3));
    com_lnk_3R_Tf = trvec2tform(rob_mech.right.q3_hip_pitch.body_rt.com);
    setFixedTransform(com_lnk_3R.Joint,com_lnk_3R_Tf);
    
    %% LINK 4
    % define the first link of the left leg
    link_list.lleg(4) = "Link4L";
    lnk_4L = rigidBody(link_list.lleg(4));
    %define the rotational joint and set home position
    joint_list.lleg(4) = "LefKneePitch";
    jnt_4L = rigidBodyJoint(joint_list.lleg(4),"revolute");
    jnt_4L.HomePosition = rob_mech.left.q4_knee_pitch.qinit; %why negative angle? see next explaination
    %define the relative transformation (the conceptual model has the same
    %joint orientation for both HipPitch as well as KneePitch, that is why
    %the rot_2 property is used istead of rot which would rotate the CF
    %around x)
    jnt_4L_Tf = trvec2tform(rob_mech.left.q4_knee_pitch.pos_virt)*eul2tform(rob_mech.left.q4_knee_pitch.rot_2,eul_ord);
    setFixedTransform(jnt_4L,jnt_4L_Tf);
    %connect the joint to the link
    lnk_4L.Joint = jnt_4L;
    %add visuals to the body
    addVisual(lnk_4L,"Mesh","Link4L_rtbox_rem.stl",eul2tform([0,0,0],eul_ord));  
    
    %left link 4 com frame
    com_list.lleg(4) = "Link4LCom";
    com_lnk_4L = rigidBody(com_list.lleg(4));
    com_lnk_4L_Tf = trvec2tform(rob_mech.left.q4_knee_pitch.body_rt.com);
    setFixedTransform(com_lnk_4L.Joint,com_lnk_4L_Tf);
    
    
    % define the first link of the right leg
    link_list.rleg(4) = "Link4R";
    lnk_4R = rigidBody(link_list.rleg(4));
    %define the rotational joint and set home position
    joint_list.rleg(4) = "RigKneePitch";
    jnt_4R = rigidBodyJoint(joint_list.rleg(4),"revolute");
    jnt_4R.HomePosition = rob_mech.right.q4_knee_pitch.qinit; %why negative angle? see next explaination
    %define the relative transformation (the conceptual model has the same
    %joint orientation for both HipPitch as well as KneePitch, that is why
    %the rot_2 property is used istead of rot which would rotate the CF
    %around x)
    jnt_4R_Tf = trvec2tform(rob_mech.right.q4_knee_pitch.pos_virt)*eul2tform(rob_mech.right.q4_knee_pitch.rot_2,eul_ord);
    setFixedTransform(jnt_4R,jnt_4R_Tf);
    %connect the joint to the link
    lnk_4R.Joint = jnt_4R;
    %add visuals to the body
    addVisual(lnk_4R,"Mesh","Link4R_rtbox_rem.stl");  
    
    %right link 4 com frame
    com_list.rleg(4) = "Link4RCom";
    com_lnk_4R = rigidBody(com_list.rleg(4));
    com_lnk_4R_Tf = trvec2tform(rob_mech.right.q4_knee_pitch.body_rt.com);
    setFixedTransform(com_lnk_4R.Joint,com_lnk_4R_Tf);
    
    
    %% LINK 5
    % define the first link of the left leg
    link_list.lleg(5) = "Link5L";
    lnk_5L = rigidBody(link_list.lleg(5));
    %define the rotational joint and set home position
    joint_list.lleg(5) = "LefAnklePitch";
    jnt_5L = rigidBodyJoint(joint_list.lleg(5),"revolute");
    jnt_5L.HomePosition = rob_mech.left.q5_ankle_pitch.qinit;
    %define the relative transformation
    jnt_5L_Tf = trvec2tform(rob_mech.left.q5_ankle_pitch.pos)*eul2tform(rob_mech.left.q5_ankle_pitch.rot,eul_ord);
    setFixedTransform(jnt_5L,jnt_5L_Tf);
    %connect the joint to the link
    lnk_5L.Joint = jnt_5L;
    %add visuals to the body
    addVisual(lnk_5L,"Mesh","Link5L_rem.stl");  
    
    %left link 5 com frame
    com_list.lleg(5) = "Link5LCom";
    com_lnk_5L = rigidBody(com_list.lleg(5));
    com_lnk_5L_Tf = trvec2tform(rob_mech.left.q5_ankle_pitch.body_rt.com);
    setFixedTransform(com_lnk_5L.Joint,com_lnk_5L_Tf);
    
    
    % define the first link of the right leg
    link_list.rleg(5) = "Link5R";
    lnk_5R = rigidBody(link_list.rleg(5));
    %define the rotational joint and set home position
    joint_list.rleg(5) = "RigAnklePitch";
    jnt_5R = rigidBodyJoint(joint_list.rleg(5),"revolute");
    jnt_5R.HomePosition = rob_mech.right.q5_ankle_pitch.qinit;
    %define the relative transformation
    jnt_5R_Tf = trvec2tform(rob_mech.right.q5_ankle_pitch.pos)*eul2tform(rob_mech.right.q5_ankle_pitch.rot,eul_ord);
    setFixedTransform(jnt_5R,jnt_5R_Tf);
    %connect the joint to the link
    lnk_5R.Joint = jnt_5R;
    %add visuals to the body
    addVisual(lnk_5R,"Mesh","Link5R_rem.stl");  

    %right link 5 com frame
    com_list.rleg(5) = "Link5RCom";
    com_lnk_5R = rigidBody(com_list.rleg(5));
    com_lnk_5R_Tf = trvec2tform(rob_mech.right.q5_ankle_pitch.body_rt.com);
    setFixedTransform(com_lnk_5R.Joint,com_lnk_5R_Tf);
    
    
    %% LINK 6
    % define the first link of the left leg
    link_list.lleg(6) = "Link6L";
    lnk_6L = rigidBody(link_list.lleg(6));
    %define the rotational joint and set home position
    joint_list.lleg(6) = "LefAnkleRoll";
    jnt_6L = rigidBodyJoint(joint_list.lleg(6),"revolute");
    jnt_6L.HomePosition = rob_mech.left.q6_ankle_roll.qinit;
    %define the relative transformation
    jnt_6L_Tf = trvec2tform(rob_mech.left.q6_ankle_roll.pos)*eul2tform(rob_mech.left.q6_ankle_roll.rot,eul_ord);
    setFixedTransform(jnt_6L,jnt_6L_Tf);
    %connect the joint to the link
    lnk_6L.Joint = jnt_6L;
    %add visuals to the body
    addVisual(lnk_6L,"Mesh","Link6_rem.stl");  
    
    %left link 6 com frame
    com_list.lleg(6) = "Link6LCom";
    com_lnk_6L = rigidBody(com_list.lleg(6));
    com_lnk_6L_Tf = trvec2tform(rob_mech.left.q6_ankle_roll.body_rt.com);
    setFixedTransform(com_lnk_6L.Joint,com_lnk_6L_Tf);
    
    
    % define the first link of the right leg
    link_list.rleg(6) = "Link6R";
    lnk_6R = rigidBody(link_list.rleg(6));
    %define the rotational joint and set home position
    joint_list.rleg(6) = "RigAnkleRoll";
    jnt_6R = rigidBodyJoint(joint_list.rleg(6),"revolute");
    jnt_6R.HomePosition = rob_mech.right.q6_ankle_roll.qinit;
    %define the relative transformation
    jnt_6R_Tf = trvec2tform(rob_mech.right.q6_ankle_roll.pos)*eul2tform(rob_mech.right.q6_ankle_roll.rot,eul_ord);
    setFixedTransform(jnt_6R,jnt_6R_Tf);
    %connect the joint to the link
    lnk_6R.Joint = jnt_6R;
    %add visuals to the body
    addVisual(lnk_6R,"Mesh","Link6_rem.stl");
    
    %right link 6 com frame
    com_list.rleg(6) = "Link6RCom";
    com_lnk_6R = rigidBody(com_list.rleg(6));
    com_lnk_6R_Tf = trvec2tform(rob_mech.right.q6_ankle_roll.body_rt.com);
    setFixedTransform(com_lnk_6R.Joint,com_lnk_6R_Tf);
    
    
    %% END EFFECTOR
    %left leg
    link_list.lleg(7) = "EE_L";
    end_eff_L = rigidBody(link_list.lleg(7));
    end_eff_L_Tf = trvec2tform(rob_mech.left.ee.pos)*eul2tform(rob_mech.left.ee.rot,eul_ord);
    setFixedTransform(end_eff_L.Joint,end_eff_L_Tf);
    %right leg
    link_list.rleg(7) = "EE_R";
    end_eff_R = rigidBody(link_list.rleg(7));
    end_eff_R_Tf = trvec2tform(rob_mech.right.ee.pos)*eul2tform(rob_mech.right.ee.rot,eul_ord);
    setFixedTransform(end_eff_R.Joint,end_eff_R_Tf);
    
    %% DYNAMIC MODEL
    %direction of the gravity
    robot_model.Gravity = [0,0,-9.81];
    
    %% !! REMARK -> default mass and inertia values for bodies are not set to zero! 
    % (all COM frames and EE frames have to be adapted in this regard)
    
    %% BASE (not really necessary)
    lnk_base.Mass = rob_mech.base.body_rt.mass;
    lnk_base.CenterOfMass = rob_mech.base.body_rt.com;
    lnk_base.Inertia = rob_mech.base.body_rt.inertia;
    %com masses and inertias have to be set to zero
    com_base.Mass = 0;
    com_base.Inertia = zeros(6,1);
    
    
    %% LINK 1
    %left leg
    lnk_1L.Mass = rob_mech.left.q1_hip_yaw.body_rt.mass;
    lnk_1L.CenterOfMass = rob_mech.left.q1_hip_yaw.body_rt.com;
    lnk_1L.Inertia = rob_mech.left.q1_hip_yaw.body_rt.inertia;
    %right leg
    lnk_1R.Mass = rob_mech.right.q1_hip_yaw.body_rt.mass;
    lnk_1R.CenterOfMass = rob_mech.right.q1_hip_yaw.body_rt.com;
    lnk_1R.Inertia = rob_mech.right.q1_hip_yaw.body_rt.inertia;
    
    %com masses and inertias have to be set to zero
    com_lnk_1L.Mass = 0;
    com_lnk_1L.Inertia = zeros(6,1);
    com_lnk_1R.Mass = 0;
    com_lnk_1R.Inertia = zeros(6,1);
    
    %% LINK 2
    %left leg
    lnk_2L.Mass = rob_mech.left.q2_hip_roll.body_rt.mass;
    lnk_2L.CenterOfMass = rob_mech.left.q2_hip_roll.body_rt.com;
    lnk_2L.Inertia = rob_mech.left.q2_hip_roll.body_rt.inertia;
    %right leg
    lnk_2R.Mass = rob_mech.right.q2_hip_roll.body_rt.mass;
    lnk_2R.CenterOfMass = rob_mech.right.q2_hip_roll.body_rt.com;
    lnk_2R.Inertia = rob_mech.right.q2_hip_roll.body_rt.inertia;
    
    %com masses and inertias have to be set to zero
    com_lnk_2L.Mass = 0;
    com_lnk_2L.Inertia = zeros(6,1);
    com_lnk_2R.Mass = 0;
    com_lnk_2R.Inertia = zeros(6,1);
    
    %% LINK 3
    %left leg
    lnk_3L.Mass = rob_mech.left.q3_hip_pitch.body_rt.mass;
    lnk_3L.CenterOfMass = rob_mech.left.q3_hip_pitch.body_rt.com;
    lnk_3L.Inertia = rob_mech.left.q3_hip_pitch.body_rt.inertia;
    %right leg
    lnk_3R.Mass = rob_mech.right.q3_hip_pitch.body_rt.mass;
    lnk_3R.CenterOfMass = rob_mech.right.q3_hip_pitch.body_rt.com;
    lnk_3R.Inertia = rob_mech.right.q3_hip_pitch.body_rt.inertia;
    
    %com masses and inertias have to be set to zero
    com_lnk_3L.Mass = 0;
    com_lnk_3L.Inertia = zeros(6,1);
    com_lnk_3R.Mass = 0;
    com_lnk_3R.Inertia = zeros(6,1);
    
    %% LINK 4
    %left leg
    lnk_4L.Mass = rob_mech.left.q4_knee_pitch.body_rt.mass;
    lnk_4L.CenterOfMass = rob_mech.left.q4_knee_pitch.body_rt.com;
    lnk_4L.Inertia = rob_mech.left.q4_knee_pitch.body_rt.inertia;
    %right leg
    lnk_4R.Mass = rob_mech.right.q4_knee_pitch.body_rt.mass;
    lnk_4R.CenterOfMass = rob_mech.right.q4_knee_pitch.body_rt.com;
    lnk_4R.Inertia = rob_mech.right.q4_knee_pitch.body_rt.inertia;
    
    %com masses and inertias have to be set to zero
    com_lnk_4L.Mass = 0;
    com_lnk_4L.Inertia = zeros(6,1);
    com_lnk_4R.Mass = 0;
    com_lnk_4R.Inertia = zeros(6,1);
    
    %% LINK 5
    %left leg
    lnk_5L.Mass = rob_mech.left.q5_ankle_pitch.body_rt.mass;
    lnk_5L.CenterOfMass = rob_mech.left.q5_ankle_pitch.body_rt.com;
    lnk_5L.Inertia = rob_mech.left.q5_ankle_pitch.body_rt.inertia;
    %right leg
    lnk_5R.Mass = rob_mech.right.q5_ankle_pitch.body_rt.mass;
    lnk_5R.CenterOfMass = rob_mech.right.q5_ankle_pitch.body_rt.com;
    lnk_5R.Inertia = rob_mech.right.q5_ankle_pitch.body_rt.inertia;
    
    %com masses and inertias have to be set to zero
    com_lnk_5L.Mass = 0;
    com_lnk_5L.Inertia = zeros(6,1);
    com_lnk_5R.Mass = 0;
    com_lnk_5R.Inertia = zeros(6,1);
    
    %% LINK 6
    %left leg
    lnk_6L.Mass = rob_mech.left.q6_ankle_roll.body_rt.mass;
    lnk_6L.CenterOfMass = rob_mech.left.q6_ankle_roll.body_rt.com;
    lnk_6L.Inertia = rob_mech.left.q6_ankle_roll.body_rt.inertia;
    %right leg
    lnk_6R.Mass = rob_mech.right.q6_ankle_roll.body_rt.mass;
    lnk_6R.CenterOfMass = rob_mech.right.q6_ankle_roll.body_rt.com;
    lnk_6R.Inertia = rob_mech.right.q6_ankle_roll.body_rt.inertia;
    
    %com masses and inertias have to be set to zero
    com_lnk_6L.Mass = 0;
    com_lnk_6L.Inertia = zeros(6,1);
    com_lnk_6R.Mass = 0;
    com_lnk_6R.Inertia = zeros(6,1);
    
    %com masses and inertias have to be set to zero for EE frames
    end_eff_L.Mass = 0;
    end_eff_L.Inertia = zeros(6,1);
    end_eff_R.Mass = 0;
    end_eff_R.Inertia = zeros(6,1);
    
    %% ATTACH THE BODIES TO THE ROBOT (THE LINKS HAVE TO BE FINALISED BY THIS POINT)
    %attach the fixed base joint to the world frame
    addBody(robot_model,lnk_base,"base");
    
    
    %link 1
    addBody(robot_model,lnk_1L,link_list.base);
    addBody(robot_model,lnk_1R,link_list.base);    
    
    %link 2
    addBody(robot_model,lnk_2L,link_list.lleg(1));
    addBody(robot_model,lnk_2R,link_list.rleg(1));
    
    %link 3
    addBody(robot_model,lnk_3L,link_list.lleg(2));
    addBody(robot_model,lnk_3R,link_list.rleg(2));
    
    %link 4
    addBody(robot_model,lnk_4L,link_list.lleg(3));
    addBody(robot_model,lnk_4R,link_list.rleg(3));
    
    %link 5
    addBody(robot_model,lnk_5L,link_list.lleg(4));
    addBody(robot_model,lnk_5R,link_list.rleg(4));
    
    %link 6
    addBody(robot_model,lnk_6L,link_list.lleg(5));
    addBody(robot_model,lnk_6R,link_list.rleg(5));
    
    robot_model_dyn = robot_model.copy;
    
    %end effector (just for visualisation purposes)
    addBody(robot_model,end_eff_L,link_list.lleg(6));
    addBody(robot_model,end_eff_R,link_list.rleg(6));
    
    %coms
    addBody(robot_model,com_base,link_list.base); %com
    addBody(robot_model,com_lnk_1L,link_list.lleg(1)); %com
    addBody(robot_model,com_lnk_1R,link_list.rleg(1)); %com
    addBody(robot_model,com_lnk_2L,link_list.lleg(2)); %com
    addBody(robot_model,com_lnk_2R,link_list.rleg(2)); %com
    addBody(robot_model,com_lnk_3L,link_list.lleg(3)); %com
    addBody(robot_model,com_lnk_3R,link_list.rleg(3)); %com
    addBody(robot_model,com_lnk_4L,link_list.lleg(4)); %com
    addBody(robot_model,com_lnk_4R,link_list.rleg(4)); %com
    addBody(robot_model,com_lnk_5L,link_list.lleg(5)); %com
    addBody(robot_model,com_lnk_5R,link_list.rleg(5)); %com
    addBody(robot_model,com_lnk_6L,link_list.lleg(6)); %com
    addBody(robot_model,com_lnk_6R,link_list.rleg(6)); %com
    
    %% COLLISIONS
    Nlnk = 13;
    lnk_order = [1,1,1,2,2,3,3,4,4,5,5,6,6];
    str_type = ["B","L","R","L","R","L","R","L","R","L","R","L","R"];
    
    %get simple link bounding geometries
    col_struct = get_collision_geom();
    
    %traverse all the links and add collision geometries
    for i=1:Nlnk
        %determine, which link should be checked
        if str_type(i) == "B"
            %base (object only has one entry)
            col_bod = col_struct.base;
        elseif str_type(i) == "L"
            %left leg
            col_bod = col_struct.lleg{lnk_order(i)};
        else
            %right leg
            col_bod = col_struct.rleg{lnk_order(i)};
        end
        
        %check the number of collision geometries belonging to the i-th link
        Ncol = col_bod.N;
        for j=1:Ncol
            %check if this is a box or a cylinder
            if col_bod.col{j}.type == "box"
                col_obj = collisionBox(col_bod.col{j}.dim(1),col_bod.col{j}.dim(2),col_bod.col{j}.dim(3)); 
            else
                col_obj = collisionCylinder(col_bod.col{j}.dim(1),col_bod.col{j}.dim(2));
            end
            %calculate the relative transformation of collision frame and apply it
            %to the object.
            obj_Tf = trvec2tform(col_bod.col{j}.pos)*eul2tform(col_bod.col{j}.rot,eul_ord);
            col_obj.Pose = obj_Tf;
            %add all the collision bounding geometry to the current link
            addCollision(robot_model.Bodies{i},col_obj);
            %show(robot_model,"Collisions","on","Visuals","on");
        end
    end
    if visualise
        show(robot_model,"Collisions","on","Visuals","on");
        %show(robot_model,"Collisions","on","Visuals","on");
    end
    
    %visualise the robot
    %vizTree = interactiveRigidBodyTree(robot_model);
    frame_list.joints = joint_list;
    frame_list.links = link_list;
    frame_list.coms = com_list;
    
end
