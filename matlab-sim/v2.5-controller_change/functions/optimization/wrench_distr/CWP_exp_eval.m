function [ineq] = CWP_exp_eval(w,param)  
    U = CWP_exp(param,0);
    ineq = U*w;
end

