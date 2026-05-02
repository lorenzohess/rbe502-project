function R = axisangle2rot(omega,theta)
    % AXISANGLE2ROT - compute the rotation matrix of an axis and angle
    %
    % Input arguments:
    % omega - the axis about which to rotate
    % theta - the angle with which to rotate about that axis
    %
    % Output arguments:
    % R - the rotation matrix

    % Use Rodrigues' formula.
    % Extract sin, cos, and the elements of omega.
    st = sin(theta);
    ct = cos(theta);
    w1 = omega(1);
    w2 = omega(2);
    w3 = omega(3);
    % Build skew-symmetric matrix.
    skewOmega = [0 -w3 w2; w3 0 -w1; -w2 w1 0];
    % The formula.
    R = eye(3) + st*skewOmega + (1 - ct)*skewOmega*skewOmega;
end
