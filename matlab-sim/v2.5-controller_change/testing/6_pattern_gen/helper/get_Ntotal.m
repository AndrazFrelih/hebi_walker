function [Ntot] = get_Ntotal(state)
    Ntot = 0;
    for i=1:size(state,1)
        Ntot = Ntot + state{i}.Npts;
    end
end

