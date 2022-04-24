function [F] = damperForceModel(d,consts)
    F0 = consts.F0;
    L0 = consts.L0;
    F = F0 * (1 - L0 * d/2);
end

