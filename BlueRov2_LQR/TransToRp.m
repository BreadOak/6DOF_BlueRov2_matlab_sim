function [R, p] = TransToRp(T)
    R = T(1: 3, 1: 3);
    p = T(1: 3, 4);
end