function [Hbase, Hlist_L, Hlist_R] = get_transforms_W_com(x,qL,qR)
    NL = max(size(qL));
    NR = max(size(qR));
    Hlist_L = zeros(NL,4,4);
    Hlist_R = zeros(NR,4,4);
    
    Q = [x;qL;qR];
    
    Hbase = Hwb_com(Q);
    
    Hlist_L(1,:,:) = Hw1L_com(Q);
    Hlist_L(2,:,:) = Hw2L_com(Q);
    Hlist_L(3,:,:) = Hw3L_com(Q);
    Hlist_L(4,:,:) = Hw4L_com(Q);
    Hlist_L(5,:,:) = Hw5L_com(Q);
    Hlist_L(6,:,:) = HweeL_com(Q);
    
    Hlist_R(1,:,:) = Hw1R_com(Q);
    Hlist_R(2,:,:) = Hw2R_com(Q);
    Hlist_R(3,:,:) = Hw3R_com(Q);
    Hlist_R(4,:,:) = Hw4R_com(Q);
    Hlist_R(5,:,:) = Hw5R_com(Q);
    Hlist_R(6,:,:) = HweeR_com(Q);
end

