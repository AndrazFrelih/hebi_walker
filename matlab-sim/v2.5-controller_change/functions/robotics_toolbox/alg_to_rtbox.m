function [Qtb] = alg_to_rtbox(Qalg)
    Qtb = zeros(size(Qalg));
    Qtb(1:2:end) = Qalg(1:end/2);
    Qtb(2:2:end) = Qalg(end/2+1:end);
end

