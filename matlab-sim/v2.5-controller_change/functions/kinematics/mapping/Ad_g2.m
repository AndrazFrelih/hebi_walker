function [matrix] = Ad_g2(H)
    matrix = Ad_g(H(1:3,1:3),H(1:3,4));
end

