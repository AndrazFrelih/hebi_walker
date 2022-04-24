function [Jcom_list] = compute_com_jac_base2(Nq,Hb_list,Hb_com_list)
    % Nq - degrees of freedom of both legs NdofL + NdofR
    %
    % Hb_list - list of absolute transformations from leg CFs to the base.
    % There are 2*(Nq+1) transformations in the list (including the offset
    % of the leg base ( structure: [LLeg, RLeg] )
    %
    % Hb_com_list - list of all transformations from COM frames to the base
    % frame ( structure: [BaseCOM, LLegCOMs, RLegCOMs] )
    
    % Jcom_list is has the following structure: [BaseCOM, LLegCOMs, RLegCOMs]
    % Single jacobians are of the size (6 x 12)
    
    Ndof = 6;
    Ncom = size(Hb_com_list,1); 
    Jcom_list = zeros(Ncom,Ndof,Nq);
    
    %Jcom_list(1) corresponds to the floating base, which is not actuated
    %(Jacobian stays zero)
    HbL = Hb_list(1:end/2,:,:);
    HbR = Hb_list(end/2+1:end,:,:);
    
    Hcom_leg = Hb_com_list(2:end,:,:);
    
    %jacobians are valid for both legs (the side of the non active leg is
    %filled with zeros)
    Nj = Ncom-1;
    %Nj is not necessairly equal to Nq (if damper is included)
    for i=1:Nj
        if i<=Nj/2
            %left side
            frame = i+1;
            Jcom_list(i+1,:,1:Nj/2) = compute_jacobian2(HbL,frame,reshape(Hcom_leg(i,:,:),4,4));
            % this is the proximal variation (hence an adaptation has to be
            % made)
            if frame <= Nj/2
                Jcom_list(i+1,:,i+1) = zeros(6,1);
            end
        else
            %right side
            frame = i+1-Nj/2;
            Jcom_list(i+1,:,Nj/2+1:Nj) = compute_jacobian2(HbR,frame,reshape(Hcom_leg(i,:,:),4,4));
            if frame <= Nj/2
                Jcom_list(i+1,:,i+1) = zeros(6,1);
            end
        end
    end
end

