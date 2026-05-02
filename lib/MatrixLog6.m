function expmat = MatrixLog6(T)
    % MATRIXLOG6 - The matrix logarithm, i.e. inverse of matrix exponential.

    % Extract rotation matrix and position vector
[R, p] = TransToRp(T);
% Take matrix log of R
omgmat = MatrixLog3(R);

% Construct HT if 
if isequal(omgmat, zeros(3))
    expmat = [zeros(3), T(1: 3, 4); 0, 0, 0, 0];
else
    % 
    theta = acos((trace(R) - 1) / 2);
    expmat = [omgmat, (eye(3) - omgmat / 2 ...
                      + (1 / theta - cot(theta / 2) / 2) ...
                        * omgmat * omgmat / theta) * p;
              0, 0, 0, 0];    
end
end
