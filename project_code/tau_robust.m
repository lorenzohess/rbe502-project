function tau = tau_robust(q, qdot, qd, qdDot, qdDdot, Kp, Kv, P, rho, p_bar)
    e = qd - q;
    edot = qdDot - qdot;

    Mbar = M_fun(q, p_bar);
    Cbar = C_fun(q, qdot, p_bar);
    Gbar = G_fun(q, p_bar);

    E = [zeros(4); eye(4)];

    xi = [e; edot];
    W = 2 * E' * P * xi;

    nW = norm(W)

    epsilon = 0.25;
    % if nW > epsilon
        Delta = rho * (W / nW);
        % disp("0")
    % else
        % Delta = rho * (W / epsilon);
        % disp("1")
    % end

    tau = Mbar * (qdDdot + Delta) + Kv*edot + Kp*e + Cbar*qdot + Gbar;
end
