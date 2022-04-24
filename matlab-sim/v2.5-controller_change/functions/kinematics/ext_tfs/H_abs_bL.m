function [Habs] = H_abs_bL(qL)
    N = size(qL,1)+1;
    Habs = zeros(N,4,4);
    HLrel = Hlleg(qL);
    Hbase = Hbase_out;
    Habs(1,:,:) = Hbase(1,:,:);
    for i=2:N
        Habs(i,:,:) = reshape(Habs(i-1,:,:),4,4)*reshape(HLrel(i-1,:,:),4,4);
    end
end

