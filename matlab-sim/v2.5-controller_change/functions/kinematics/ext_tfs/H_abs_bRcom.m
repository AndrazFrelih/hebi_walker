function [Hcom] = H_abs_bRcom(HRabs,qR)
    N = size(qR,1);
    Hcom = zeros(N,4,4);
    Hcom_rel = Hrleg_com(qR);
    for i=1:N
        Hcom(i,:,:) = reshape(HRabs(i,:,:),4,4)*reshape(Hcom_rel(i,:,:),4,4);
    end
end

