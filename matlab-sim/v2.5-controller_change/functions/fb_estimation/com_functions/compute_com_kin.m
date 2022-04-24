function [C,Cp] = compute_com_kin(x,qL,qR,xp,qLp,qRp,mlist,f)
    [Hcom,HL,HR] = H_com_list(qL,qR);
    Cb = compute_com_pos(Hcom,mlist);
    
    Habs = zeros(size(HL,1)+size(HR,1),4,4);
    Habs(1:end/2,:,:) = HL;
    Habs(end/2+1:end,:,:) = HR;
    Jlist = compute_com_jac_base(12,Habs,Hcom);
    
    if f=='b'
        C = Cb;
        %only take the translational part
        Cp = compute_com_vel(Jlist(:,1:3,:),mlist,[qLp;qRp]);
    elseif f=='w'
        %transform robot com from base to world frame
        Hwb = Hbase_in(x);
        C = reshape(Hwb(1:3,1:3),3,3)*Cb + reshape(Hwb(1:3,4),3,1);
        %create special jacobian mappings (from ETH zurich teaching)
        Jlist_w = compute_jac_world(Hwb,Jlist,Hcom);
        gen_vel = FB_vel_map(x(4:6),[xp;zeros(3,1);qLp;qRp]);
        Cp = compute_com_vel(Jlist_w(:,1:3,:),mlist,gen_vel);
    end
end

