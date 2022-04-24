function [w] = gainWeight(dT, q, time,type)
    if type == 1
        dTn = (1-q)*dT; 
        w = (time - q*dT)/dTn;
    else
        dTn = q*dT; 
        w = time/dTn;
    end
end

