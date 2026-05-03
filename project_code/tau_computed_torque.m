function tau = tau_computed_torque(q, qdot, qd, qdDot, qdDdot, Kp, Kv, p, pf)
    e = qd - q;
    edot = qdDot - qdot;

    M = M_fun(q, p);
    C = C_fun(q, qdot, p);
    G = G_fun(q, p);

    % tau_friction = ViscousFriction_fun(qdot, pf);

    tau = M * qdDdot + Kv*edot + Kp*e + C*qdot + G;
end
