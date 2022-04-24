function [Hbase, Hlist_L, Hlist_R] = get_transforms_B_com(qL,qR)
    NL = max(size(qL));
    NR = max(size(qR));
    Hlist_L = zeros(NL,4,4);
    Hlist_R = zeros(NR,4,4);
    
    Q = [qL;qR];
    
    Hbase = Hbb_com(Q); %base com frame to base frame
    
    Hlist_L(1,:,:) = Hb1L_com(Q);
    Hlist_L(2,:,:) = Hb2L_com(Q);
    Hlist_L(3,:,:) = Hb3L_com(Q);
    Hlist_L(4,:,:) = Hb4L_com(Q);
    Hlist_L(5,:,:) = Hb5L_com(Q);
    Hlist_L(6,:,:) = HbeeL_com(Q);
    
    Hlist_R(1,:,:) = Hb1R_com(Q);
    Hlist_R(2,:,:) = Hb2R_com(Q);
    Hlist_R(3,:,:) = Hb3R_com(Q);
    Hlist_R(4,:,:) = Hb4R_com(Q);
    Hlist_R(5,:,:) = Hb5R_com(Q);
    Hlist_R(6,:,:) = HbeeR_com(Q);
end

