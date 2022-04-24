X = rand(18,1);
mlist = rand(13,1);
[Jw_com, Jw_eeL, Jw_eeR, Jw_b] = get_jacobians_time_test(X,mlist);