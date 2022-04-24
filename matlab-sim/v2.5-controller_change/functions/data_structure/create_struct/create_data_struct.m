function [lleg_str, rleg_str, lleg_FS_str, rleg_FS_str] = create_data_struct(qL_ini, qR_ini,total_mass)
    % motor data
    mot_L = cell(6,1);
    mot_R = cell(6,1);
    g = 9.81;
    for i=1:6
        mot_L{i} = struct('q',qL_ini(i),'qp',0,'a',[zeros(2,1);g],'w',zeros(3,1));
        mot_R{i} = struct('q',qR_ini(i),'qp',0,'a',[zeros(2,1);g],'w',zeros(3,1));
    end
    
    lleg_str.mot1 = mot_L{1};
    lleg_str.mot2 = mot_L{2};
    lleg_str.mot3_1 = mot_L{3};
    lleg_str.mot3_2 = mot_L{3};
    lleg_str.mot3_2.q = -lleg_str.mot3_2.q; 
    lleg_str.mot4_1 = mot_L{4};
    lleg_str.mot4_1.q = -lleg_str.mot4_1.q;
    lleg_str.mot4_2 = mot_L{4};
    lleg_str.mot5 = mot_L{5};
    lleg_str.mot6 = mot_L{6};
    
    rleg_str.mot1 = mot_R{1};
    rleg_str.mot2 = mot_R{2};
    rleg_str.mot3_1 = mot_R{3};
    rleg_str.mot3_2 = mot_R{3};
    rleg_str.mot3_2.q = -rleg_str.mot3_2.q;
    rleg_str.mot4_1 = mot_R{4};
    rleg_str.mot4_1.q = -rleg_str.mot4_1.q;
    rleg_str.mot4_2 = mot_R{4};
    rleg_str.mot5 = mot_R{5};
    rleg_str.mot6 = mot_R{6};

    %force sensor data
    mass_per_leg = total_mass/2;
    mass_per_sensor = mass_per_leg/4;
    lleg_FS_str = struct('Fm',mass_per_sensor*ones(4,1)*9.81,'state',4);
    rleg_FS_str = lleg_FS_str;
end

