function MM = MassMatrixCalculator(theta, S, M, G)
% Compute mass matrix using \sum_i^n J'*G*J formula.
% Params:
%  theta - nx1 joint angles
%  S - 6xn screw axes in space frame
%  M - 4x4x(n+1), poses of each link, including EE, relative to previous
%  link
%  G - 6x6xn spatial inertia matrix
%
% J is the body Jacobian of the ith-link in the chain -- NOT of the
% end effector.

% Initialize the mass matrix
n = size(S,2);
MM = zeros(n,n);

for linkIdx = 1:n
    % Compute Mhome
    Mhome = eye(4);
    for ii = 1:linkIdx
        Mhome = Mhome * M(:,:,ii);
    end

    % Compute HT from {0} to {i}
    T0i = eye(4);
    for iii = 1:linkIdx
        T = twist2ht(S(:,iii), theta(iii));
        T0i = T0i * T;
    end
    T0i = T0i * Mhome;
    Ti0 = TransInv(T0i);

    % Convert S to body frame. We must convert each joint's screw axis
    % to body, to iterate from 1:n.
    B = zeros(6,n);
    for screwAxisIdx = 1:n
        B(:,screwAxisIdx) = adjoint(inv(Mhome)) * S(:,screwAxisIdx);
    end

    % Compute Mhome
    Jbody = zeros(6,n);
    for JbcolIdx = 1:linkIdx % only partial Jacobian
        T = eye(4);
        for jj = linkIdx:-1:(JbcolIdx+1) % PoE backwards
            T = T * twist2ht(-B(:,jj), theta(jj));
        end
        Jbody(:,JbcolIdx) = adjoint(T) * B(:,JbcolIdx);
    end

    % Compute MM
    Gi = G(:,:,linkIdx);
    MM = MM + Jbody' * Gi * Jbody;
end
end
