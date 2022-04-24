function [Hlist_L, Hlist_R] = get_transforms_B(qL,qR)
    NL = max(size(qL))+1;
    NR = max(size(qR))+1;
    Hlist_L = zeros(NL,4,4);
    Hlist_R = zeros(NR,4,4);
    
    Q = [qL;qR];
    
    Hlist_L(1,:,:) = Hb0L(Q);
    Hlist_L(2,:,:) = Hb1L(Q);
    Hlist_L(3,:,:) = Hb2L(Q);
    Hlist_L(4,:,:) = Hb3L(Q);
    Hlist_L(5,:,:) = Hb4L(Q);
    Hlist_L(6,:,:) = Hb5L(Q);
    Hlist_L(7,:,:) = HbeeL(Q);
    
    Hlist_R(1,:,:) = Hb0R(Q);
    Hlist_R(2,:,:) = Hb1R(Q);
    Hlist_R(3,:,:) = Hb2R(Q);
    Hlist_R(4,:,:) = Hb3R(Q);
    Hlist_R(5,:,:) = Hb4R(Q);
    Hlist_R(6,:,:) = Hb5R(Q);
    Hlist_R(7,:,:) = HbeeR(Q);
end

