function [KpL, KpR, KdL, KdR] = get_gains4(test_type, wGain)
    % SS swing foot
    Kp_swing = [200,250,200,320,30,4]';
    Kd_swing = [10,25,15,20,3,1.5]';
    
    % SS stance foot
    Kp_ds = [600,2000,1000,1250,250,50]'*3;
    Kd_ds = [20,20,20,20,7,3]';
    
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
        wL = max(min(wGain(1),1),0);
        wR = max(min(wGain(2),1),0);
        
        N = 6;
        KpL = zeros(N,1);
        KpR = zeros(N,1);
        KdL = zeros(N,1);
        KdR = zeros(N,1);
        
        for i=1:N
            KpL(i) = interp1([0,1],[Kp_ds(i),Kp_swing(i)],wL);
            KpR(i) = interp1([0,1],[Kp_ds(i),Kp_swing(i)],wR);
            KdL(i) = interp1([0,1],[Kd_ds(i),Kd_swing(i)],wL);
            KdR(i) = interp1([0,1],[Kd_ds(i),Kd_swing(i)],wR);
        end
    end
end

