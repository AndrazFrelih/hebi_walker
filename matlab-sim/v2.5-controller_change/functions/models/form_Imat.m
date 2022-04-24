function [Imat] = form_Imat(Ivec)
    Imat = [
        Ivec(1), Ivec(4), Ivec(5);
        Ivec(4), Ivec(2), Ivec(6);
        Ivec(5), Ivec(6), Ivec(3)
    ];
end

