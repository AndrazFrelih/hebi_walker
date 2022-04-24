function [TL, TR] = damperCompensTorques2(qL, qR, HL, HR, consts)
    %% get relative damper attachment transformations:
    % left
    HLhip_rel = HLdamp_hip(qL(2));
    HLknee_rel = HLdamp_knee(qL(4));    
    % right
    HRhip_rel = HRdamp_hip(qR(2));
    HRknee_rel = HRdamp_knee(qR(4));
    
    %% frame replaced by the transformation:
    frame_hip = 3;
    frame_knee = 5;

    %% compute absolute damper attachment transformations:
    % left
    HLhip = reshape(HL(frame_hip-1,:,:),4,4) * HLhip_rel;
    HLknee = reshape(HL(frame_knee-1,:,:),4,4) * HLknee_rel;
    % right
    HRhip = reshape(HR(frame_hip-1,:,:),4,4) * HRhip_rel;
    HRknee = reshape(HR(frame_knee-1,:,:),4,4) * HRknee_rel;
    
    %% generate required lists for the jacobian computation
    % left
    HLhip_list = HL;
    HLhip_list(frame_hip,:,:) = HLhip;
    HLhip_list(frame_hip+1:end,:,:) = 0;
    HLknee_list = HL;
    HLknee_list(frame_knee,:,:) = HLknee;
    HLknee_list(frame_knee+1:end,:,:) = 0;
    % right
    HRhip_list = HR;
    HRhip_list(frame_hip,:,:) = HRhip;
    HRhip_list(frame_hip+1:end,:,:) = 0;
    HRknee_list = HR;
    HRknee_list(frame_knee,:,:) = HRknee;
    HRknee_list(frame_knee+1:end,:,:) = 0;

    %% compute jacobians from attachments to the base:
    % left
    JLhip = compute_jacobian(HLhip_list,frame_hip);
    JLhip(:,frame_hip) = 0; %proximal implementation bug...
    JLknee = compute_jacobian(HLknee_list,frame_knee);
    JLknee(:,frame_knee) = 0; %proximal implementation bug...
    % right
    JRhip = compute_jacobian(HRhip_list,frame_hip);
    JRhip(:,frame_hip) = 0; %proximal implementation bug...
    JRknee = compute_jacobian(HRknee_list,frame_knee);
    JRknee(:,frame_knee) = 0; %proximal implementation bug...

    %% compute static forces:
    % left
    FL = damperComputeForce(HLhip,HLknee,consts);
    % right
    FR = damperComputeForce(HRhip,HRknee,consts);

    %% map forces to torques:
    % left
    TL = (JLknee(1:3,:)-JLhip(1:3,:))'*FL;
    % right
    TR = (JRknee(1:3,:)-JRhip(1:3,:))'*FR;
end

