angle_vec = zeros(12,1);
angle_vec(3) = -pi/4;
angle_vec(4) = pi/4;

angle_vec(12) = pi/4;

show(robot_model_dyn,angle_vec);

%L1 ok 1
%R1 ok 2 
%L2 ok 3
%R2 ok 4
%L3 ok 5
%R3 ok 6
%L4 ok 7
%R4 ok 8
%L5 ok 9
%R5 ok 10
%L6 ok 11
%R6 ok 12