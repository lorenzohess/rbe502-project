function tau = tau_robust(q, qdot, qd, qdDot, qdDdot, Kp, Kv, P, rho, p_bar)
    e = qd - q;
    edot = qdDot - qdot;

    Mbar = M_fun(q, p_bar);
    Cbar = C_fun(q, qdot, p_bar);
    Gbar = G_fun(q, p_bar);

    E = [zeros(4); eye(4)];
    P;
    xi = [e; edot];
    W = 2 * E' * P * xi;

    nW = norm(W);
    epsilon = 0.1;
    if nW > epsilon
        Delta = rho * (W / nW);
    else
        Delta = rho * (W / epsilon);
    end

    tau = Mbar * qdDdot + Kv*edot + Kp*e + Delta + Cbar*qdot + Gbar;
end
