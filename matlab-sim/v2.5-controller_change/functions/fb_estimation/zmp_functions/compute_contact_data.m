function [FL, FR, TL, TR, cL, cR] = compute_contact_data(FSL, FSR, stateL, stateR, ctrl_cfg)
    %% get required data
    Pmeas = [ctrl_cfg.fb_est.foot_data.fsen.Pmeas, zeros(4,1)];
    
    %% compute the total force in the foot CF
    FLtot = sum(FSL.Fm);
    FRtot = sum(FSR.Fm);
    
    %% compute the torque in the foot CF
    TLtot = zeros(3,1);
    TRtot = zeros(3,1);
    for i=1:4
        TLtot = TLtot + cross(Pmeas(i,:),[0,0,FSL.Fm(i)])';
        TRtot = TRtot + cross(Pmeas(i,:),[0,0,FSR.Fm(i)])';
    end
    
    %% determine the contact state
    % use the force sensor data to determine the contact mode of feet (0
    % not in full contact, 1 in full contact)
    Fthr = 100; % threshold contact force
    if stateL == 4 && FLtot>Fthr
        contL = 1;
    else
        contL = 0;
    end

    if stateR == 4 && FRtot>Fthr
        contR = 1;
    else
        contR = 0;
    end
    
    FL = [0;0;FLtot];
    FR = [0;0;FRtot];
    TL = TLtot;
    TR = TRtot;
    cL = contL;
    cR = contR;
end

