function [err_vec] = get_com_error(x,qL,qR,Cdes,mlist) 
    xfull = [
        x;          %only position of the fb is important
        zeros(3,1); %base has to be upright
    ];
    Cw = compute_com_pos2(xfull,qL,qR,mlist,'w');
    err_vec = Cdes - Cw;
end

