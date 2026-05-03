function [tau, pi_hat_new, pi_hat_dot] = tau_adaptive(q, qdot, ...
                                                      qd, qdDot, qdDdot, ...
                                                      Kp, Kv, P, R_inv, ...
                                                      pi_hat, p_bar, dt)
% Adaptive computed-torque controller.
%   tau = Y(q,qdot,a,p_bar) * pi_hat
%   pi_hat_dot = R^{-1} Y(q,qdot,qddot,p_bar)^T B_hat^{-1} E^T P xi
%   B_hat(q) = M_hat(q) built by column extraction from Y.
% pi_hat: 16x1 (m1..m4, Ixx1..Izz4), maps to p_bar(7:22).

    n = length(q);

    % Errors
    e    = qd    - q;
    edot = qdDot - qdot;
    xi   = [e; edot];


    % B_hat(q) via column extraction
    zer    = zeros(n,1);
    Y_grav = Y_fun(q, zer, zer, p_bar);
    B_hat  = zeros(n);
    for jj = 1:n
        ej         = zer; ej(jj) = 1;
        Y_j        = Y_fun(q, zer, ej, p_bar);
        B_hat(:,jj) = (Y_j - Y_grav) * pi_hat;
    end
    
    % Reference acceleration
    a = qdDdot + pinv(B_hat)*Kv*edot + pinv(B_hat)*Kp*e;

    % Control law
    Y_a = Y_fun(q, qdot, a, p_bar);
    tau = Y_a * pi_hat;
    
    E          = [zeros(n); eye(n)];
    Y_dyn      = Y_fun(q, qdot, qdDdot, p_bar);
    pi_hat_dot = R_inv * Y_dyn' * pinv(B_hat)' * E' * P * xi;

    % Euler integration
    pi_hat_new = pi_hat + pi_hat_dot * dt;
end
