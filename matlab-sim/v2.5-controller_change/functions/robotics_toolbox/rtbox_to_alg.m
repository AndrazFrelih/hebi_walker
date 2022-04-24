function [Qalg] = rtbox_to_alg(Qtb)
    Qalg = zeros(size(Qtb));
    Qalg(1:end/2) = Qtb(1:2:end);
    Qalg(end/2+1:end) = Qtb(2:2:end);
end

