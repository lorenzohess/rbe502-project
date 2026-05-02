function traj = make_trajectory(type, params)
traj = struct();

ti = params.t(1);
tf = params.t(2);
t = ti:params.time_step:tf;
traj.t = t;

qi = params.q(1);
qf = params.q(2);

qdi = params.v(1);
qdf = params.v(2);

switch type
    case 'cubic'
        T = [1 ti ti^2 ti^3; 0 1 2*ti 3*ti^2; 1 tf tf^2 tf^3; 0 1 2*tf 3*tf^2];
        Q = [qi; qdi; qf; qdf];
        A = T \ Q;
        a0 = A(1);
        a1 = A(2);
        a2 = A(3);
        a3 = A(4);
        traj.q = a0 + a1*t + a2*t.*t + a3*t.*t.*t;
        traj.v = a1 + 2*a2*t + 3*a3*t.*t;
        traj.a = 2*a2 + 6*a3*t;
    case 'quintic'
        qddi = params.a(1);
        qddf = params.a(2);
        T = [1 ti ti^2 ti^3 ti^4 ti^5;...
            0 1 2*ti 3*ti^2 4*ti^3 5*ti^4;...
            0 0 2 6*ti 12*ti^2 20*ti^3;...
            1 tf tf^2 tf^3 tf^4 tf^5;...
            0 1 2*tf 3*tf^2 4*tf^3 5*tf^4;...
            0 0 2 6*tf 12*tf^2 20*tf^3];

        Q = [qi; qdi; qddi; qf; qdf; qddf];
        A = T \ Q;

        a0 = A(1);
        a1 = A(2);
        a2 = A(3);
        a3 = A(4);
        a4 = A(5);
        a5 = A(6);

        traj.q = a0 + a1*t + a2*t.^2 + a3*t.^3 + a4*t.^4 + a5*t.^5;
        traj.v = a1 + 2*a2*t + 3*a3*t.^2 + 4*a4*t.^3 + 5*a5*t.^4;
        traj.a = 2*a2 + 6*a3*t + 12*a4*t.^2 + 20*a5*t.^3;
end

end

