function [Hcom,HL,HR] = H_com_list(qL,qR)
    Hcom = zeros(13,4,4);
    Hcom(1,:,:) = Hbase_com;
    HL = H_abs_bL(qL);
    Hcom(2:7,:,:) = H_abs_bLcom(HL,qL);
    HR = H_abs_bR(qR);
    Hcom(8:end,:,:) = H_abs_bRcom(HR,qR);
end

