function [Hcom] = H_abs_bLcom(HLabs,qL)
    N = size(qL,1);
    Hcom = zeros(N,4,4);
    Hcom_rel = Hlleg_com(qL);
    for i=1:N
        Hcom(i,:,:) = reshape(HLabs(i,:,:),4,4)*reshape(Hcom_rel(i,:,:),4,4);
    end
end

