function [tau, pi_hat_new, pi_hat_dot] = tau_adaptive(q, qdot, qddot, ...
                                                      qd, qdDot, qdDdot, ...
                                                      Kp, Kv, P, R_gain, ...
                                                      pi_hat, p_bar, dt)
% Adaptive computed-torque controller.
%   tau = Y(q,qdot,a,p_bar) * pi_hat
%   pi_hat_dot = R^{-1} Y(q,qdot,qddot,p_bar)^T B_hat^{-1} E^T P xi
%   B_hat(q) = M_hat(q) built by column extraction from Y.
% pi_hat: 16x1 (m1..m4, Ixx1..Izz4), maps to p_bar(7:22).

    n = length(q);

    % Step 1 -- errors
    e    = qd    - q;
    edot = qdDot - qdot;
    xi   = [e; edot];

    % Step 3 -- reference acceleration
    a = qdDdot + Kv*edot + Kp*e;

    % Step 4 -- control law (linear in adapted parameters)
    % clc
    Y_a = Y_fun(q, qdot, a, p_bar);
    tau = Y_a * pi_hat;

    % Step 5 -- B_hat(q) = M_hat(q) via column extraction
    %   M(:,j)*pi = (Y(q,0,e_j,p) - Y(q,0,0,p)) * pi_hat
    zer    = zeros(n,1);
    Y_grav = Y_fun(q, zer, zer, p_bar);
    B_hat  = zeros(n);
    for j = 1:n
        ej         = zer; ej(j) = 1;
        Y_j        = Y_fun(q, zer, ej, p_bar);
        B_hat(:,j) = (Y_j - Y_grav) * pi_hat;
    end

    % Step 6 -- adaptation law (uses *measured/estimated* qddot)
    E          = [zeros(n); eye(n)];
    Y_dyn      = Y_fun(q, qdot, qddot, p_bar);
    s          = B_hat \ (E' * P * xi);
    pi_hat_dot = R_gain \ (Y_dyn' * s);

    % Euler integration
    pi_hat_new = pi_hat + pi_hat_dot * dt;
end
