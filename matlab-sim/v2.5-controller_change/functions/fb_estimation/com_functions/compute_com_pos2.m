function [com] = compute_com_pos2(x,qL,qR,mlist,f)
    [Hlist,~,~] = H_com_list(qL,qR);
    Cb = compute_com_pos(Hlist,mlist);
    if f=='b'
        %compute the com in the base frame
        com = Cb;
    elseif f=='w'
        %compute the com in the world frame
        Hwb = Hbase_in(x);
        com = reshape(Hwb(1:3,1:3),3,3)*Cb + reshape(Hwb(1:3,4),3,1);
    end
end

