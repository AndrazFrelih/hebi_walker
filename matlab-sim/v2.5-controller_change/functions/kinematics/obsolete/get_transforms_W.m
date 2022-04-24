function [Hlist_L, Hlist_R] = get_transforms_W(x,qL,qR)
    NL = max(size(qL))+1;
    NR = max(size(qR))+1;
    Hlist_L = zeros(NL,4,4);
    Hlist_R = zeros(NR,4,4);
    
    Q = [x;qL;qR];
    
    Hlist_L(1,:,:) = Hw0L(Q);
    Hlist_L(2,:,:) = Hw1L(Q);
    Hlist_L(3,:,:) = Hw2L(Q);
    Hlist_L(4,:,:) = Hw3L(Q);
    Hlist_L(5,:,:) = Hw4L(Q);
    Hlist_L(6,:,:) = Hw5L(Q);
    Hlist_L(7,:,:) = HweeL(Q);
    
    Hlist_R(1,:,:) = Hw0R(Q);
    Hlist_R(2,:,:) = Hw1R(Q);
    Hlist_R(3,:,:) = Hw2R(Q);
    Hlist_R(4,:,:) = Hw3R(Q);
    Hlist_R(5,:,:) = Hw4R(Q);
    Hlist_R(6,:,:) = Hw5R(Q);
    Hlist_R(7,:,:) = HweeR(Q);
end

