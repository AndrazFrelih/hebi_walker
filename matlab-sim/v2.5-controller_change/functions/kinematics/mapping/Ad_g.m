function [matrix] = Ad_g(R,t)
sk_sym = [
    0, +t(3), -t(2);
    -t(3), 0, +t(1);
    +t(2), -t(1), 0
];

matrix = [
    R, R*sk_sym;
    zeros(3), R
];
end

