function AdT = adjoint(T)
    % ADJOINT - Compute adjoint transformation of a homogeneous transformation
    %
    % Input arguments:
    % T - the homogeneous transform in SE(3)
    %
    % Output arguments:
    % AdT - the adjoint transformation of T

    % Extract R and p
    R = T(1:3,1:3);
    p = T(1:3,4);

    % Extract each element of P to compute skew-symmetric matrix of p
    p1 = p(1);
    p2 = p(2);
    p3 = p(3);
    skewP = [0 -p3 p2; p3 0 -p1; -p2 p1 0];

    % Insert R and skewP * R into the adjoint transformation.
    AdT = zeros(6,6);
    AdT(1:3,1:3) = R;
    AdT(4:6,4:6) = R;
    AdT(1:3,4:6) = 0; % explicitly denote because reader expects it
    AdT(4:6,1:3) = skewP * R;
end
