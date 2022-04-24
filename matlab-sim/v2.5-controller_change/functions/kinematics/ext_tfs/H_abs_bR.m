function [Habs] = H_abs_bR(qR)
    N = size(qR,1)+1;
    Habs = zeros(N,4,4);
    HRrel = Hrleg(qR);
    Hbase = Hbase_out;
    Habs(1,:,:) = Hbase(2,:,:);
    for i=2:N
        Habs(i,:,:) = reshape(Habs(i-1,:,:),4,4)*reshape(HRrel(i-1,:,:),4,4);
    end
end

