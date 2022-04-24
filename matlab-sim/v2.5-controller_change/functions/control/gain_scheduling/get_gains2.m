function [KpL, KpR, KdL, KdR] = get_gains2(test_type, phase, WshiftL, WshiftR)
    
    %% Available gains
    % DS
    %Kp_ds = [450,650,650,300,50,4]';
    Kp_ds = [800,950,950,950,450,220]'*2;
    Kd_ds = [20,35,20,20,10,6]';
    %Kp_ds = [20,150,150,100,20,4]';
    %Kd_ds = [5,30,60,20,5,1]';
    %Kd_ds = sqrt(Kp_ds)*2;
    
    % SS swing foot
    Kp_swing = [40,300,350,220,15,3]';
    Kd_swing = [20,35,20,20,2,1]';
    
    % SS stance foot
    Kp_support = [800,950,950,1150,450,220]'*4;
    Kd_support = [20,35,20,20,10,6]';
    
    %% Consider different cases
    if test_type == 0 || test_type == 1
        KpL = Kp_ds;
        KpR = Kp_ds;
        KdL = Kd_ds;
        KdR = Kd_ds;
    else
        if phase == 0
            % double support phase
            KpL = Kp_ds;
            KpR = Kp_ds;
            KdL = Kd_ds;
            KdR = Kd_ds;
        else
            % single support phase
            if WshiftL > WshiftR
                % LEFT foot is supporting, RIGHT is swinging
                KpL = Kp_support;
                KpR = Kp_swing;
                KdL = Kd_support;
                KdR = Kd_swing;
            else
                % RIGHT foot is supporting, LEFT is swinging
                KpL = Kp_swing;
                KpR = Kp_support;
                KdL = Kd_swing;
                KdR = Kd_support;
            end
        end
    end
end

