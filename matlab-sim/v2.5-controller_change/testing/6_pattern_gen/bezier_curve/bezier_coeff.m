function [coeff_vect] = bezier_coeff(n,i,tvec)
    coeff_vect = nchoosek(n,i).*tvec.^i.*(1-tvec).^(n-i);
end

