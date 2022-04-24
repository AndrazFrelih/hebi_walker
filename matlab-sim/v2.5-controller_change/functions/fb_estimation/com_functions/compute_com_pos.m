function [com] = compute_com_pos(Hlist,mlist)
    mtot = sum(mlist);
    com = zeros(3,1);
    for i=1:size(Hlist,1)
        comi = reshape(Hlist(i,1:3,4),3,1);
        mi = mlist(i);
        com = com + mi * comi;
    end
    com  = com/mtot;    
end

