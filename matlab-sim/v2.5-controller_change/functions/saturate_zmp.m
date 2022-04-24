function [v_sat] = saturate_zmp(v_inp,sat)
    i1 = v_inp<0;
    sat_sign = sat;
    sat_sign(i1) = -sat(i1);
    
    v_abs = abs(v_inp);
    
    i2 = v_abs>sat;
    v_sat = v_inp;
    v_sat(i2) = sat_sign(i2);
end

