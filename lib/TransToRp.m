function [R, p] = TransToRp(T)
    % TRANSTORP - Extract rotation matrix and position from HT.
    %
    % Input arguments:
    % T - the HT
    %
    % Output arguments:
    % R - the rotation matrix
    % p - the position vector

    % By definition of HT.
R = T(1: 3, 1: 3);
p = T(1: 3, 4);
end
