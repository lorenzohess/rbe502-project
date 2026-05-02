function T = twist2ht(S,theta)
    % TWIST2HT - Convert twist-axis angle rotation to HT.
    %
    % Input arguments
    % S - the screw axis
    % theta - the angle with which to rotate about S
    %
    % Output arguments:
    % T - the corresponding HT 

    % Set up boilerplate 0s and 1
    T = zeros(4,4);
    T(4,4) = 1;

    % Extract omega and v
    omega = S(1:3);
    v = S(4:6);
    % Compute rotation matrix
    R = axisangle2rot(omega,theta);
    % Extract sin, cos, and elements of omega to make skew symmetric of omega
    st = sin(theta);
    ct = cos(theta);
    w1 = omega(1);
    w2 = omega(2);
    w3 = omega(3);
    skewOmega = [0 -w3 w2; w3 0 -w1; -w2 w1 0];
    % Compute position vector using matrix version of Rodrigues' formula
    p = (eye(3)*theta + (1-ct)*skewOmega + (theta - st)*skewOmega*skewOmega) * v;
    % Insert R and p into T
    T(1:3,1:3) = R;
    T(1:3,4) = p;
end
