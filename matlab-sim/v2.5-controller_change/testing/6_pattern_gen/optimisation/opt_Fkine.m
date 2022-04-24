function [Fkine] = opt_Fkine(Hw_ee,Hffix)
    x = Hw_ee(1,4);
    y = Hw_ee(2,4);
    
    %contact points fkine
    zcp = zeros(4,1);
    for i=1:4
       Hcp = Hw_ee*reshape(Hffix(i,:,:),4,4); 
       zcp(i) = Hcp(3,4);
    end
    
    Fkine = [
        x;
        y;
        zcp;
    ];
end

