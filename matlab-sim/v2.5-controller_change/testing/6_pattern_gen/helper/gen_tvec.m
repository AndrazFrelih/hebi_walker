function [tpts,tvec] = gen_tvec(state)
    Npts = state.Npts;
    tpts = [0,state.dT*(Npts-1)/Npts];
    tvec = linspace(tpts(1),tpts(2),state.Npts);
end

