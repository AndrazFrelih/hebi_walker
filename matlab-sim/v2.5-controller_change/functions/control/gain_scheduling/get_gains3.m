function [KpL, KpR, KdL, KdR] = get_gains3(FSM, test_type, phase, WshiftL, WshiftR, contL, contR)
    
    %% Available gains
    % DS
    Kp_ds = [800,950,950,950,250,50]'*3;
    Kd_ds = [20,35,20,20,7,3]';
    
    % SS swing foot
    Kp_swing = [200,250,200,320,30,4]';
    %Kd_swing = [10,35,30,40,3,1.5]';
    Kd_swing = [10,25,15,20,3,1.5]';
    
    % SS stance foot
    Kp_support = [600,2000,1000,1250,250,50]'*3;
    Kd_support = [20,20,20,20,7,3]';
    Kp_ds = Kp_support;
    Kd_ds = Kd_support;
    if FSM.gs.reduce_gains
        Kp_ds = Kp_swing;
        Kd_ds = Kd_swing;
        Kp_support = Kp_swing;
        Kd_support = Kd_swing;
    end
    
    %% Consider different cases
    
    if test_type == -1
        KpL = Kp_swing;
        KpR = Kp_swing;
        KdL = Kd_swing;
        KdR = Kd_swing;
    elseif test_type == 0 || test_type == 1
        KpL = Kp_ds;
        KpR = Kp_ds;
        KdL = Kd_ds;
        KdR = Kd_ds;
    else
        if phase == 0
            % double support phase
            if contL
                KpL = Kp_ds;
                KdL = Kd_ds;
            else
                % the robot has not regained contact on his left foot yet
                KpL = Kp_swing;
                KdL = Kd_swing;
            end
            
            if contR
                KpR = Kp_ds;
                KdR = Kd_ds;
            else
                % the robot has not regained contact on his right foot yet
                KpR = Kp_swing;
                KdR = Kd_swing;
            end
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

