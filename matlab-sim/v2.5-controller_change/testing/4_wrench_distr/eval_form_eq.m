function [nod] = eval_form_eq(N,param,Tmax)  
    nod = 0;
    w_set = 2*Tmax*(rand(6,N)-1/2);
    for i=1:N
        in_or = CWP_orig_eval(w_set(:,i),param);
        in_or_eval = in_or<0;
        in_ex = CWP_exp_eval(w_set(:,i),param);
        in_ex_eval = in_ex<0;
        in_ex_merge = [
            in_ex_eval(1) && in_ex_eval(2);
            in_ex_eval(3) && in_ex_eval(4);
            in_ex_eval(5) && in_ex_eval(6);
            in_ex_eval(7) && in_ex_eval(8);
            in_ex_eval(9) && in_ex_eval(10) && in_ex_eval(11) && in_ex_eval(12);
            in_ex_eval(13) && in_ex_eval(14) && in_ex_eval(15) && in_ex_eval(16);
        ];
    
        if ~isequal(in_or_eval,in_ex_merge)
            disp("Discrepancy found");
            disp("w = " + w_set(:,i));
            nod = nod + 1;
        end
    end
end