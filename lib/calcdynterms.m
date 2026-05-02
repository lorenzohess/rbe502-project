function h = calcdynterms(params, term)
  % CALCDYNTERMS - Compute specified dynamic term via RNE inverse dynamics
  %
  % Input arguments:
  % params - struct with robot state and model fields required by rne
  % term   - one of: 'centripetal-coriolis', 'gravity', 'ext-force'
  %
  % Output arguments:
  % h      - wrench/torque vector computed by rne for the requested term
function [tau,V,Vdot] = rne(params)
n = size(params.S,2);    % Number of links in the kinematic chain
S = params.S;
V = zeros(6,n+1);
Vdot = zeros(6,n+1);
V(:,1) = [0 0 0 0 0 0]';
Vdot(:,1) = [0; 0; 0; -params.g(:)];
q = params.jointPos;
qd = params.jointVel;
qdd = params.jointAcc;
Mrelative = zeros(4,4,n+1);
for i=1:n
    Mrelative(:,:,i) = inv(params.M(:,:,i));
end
Mabs = zeros(4,4,n);
Mabs(:,:,1) = params.M(:,:,1); % M1 = M01
for i=2:n
    Mabs(:,:,i) = Mabs(:,:,i-1) * params.M(:,:,i);
end
A = zeros(6,n);
for axis = 1:n
    A(:,axis) = adjoint(inv(Mabs(:,:,axis))) * S(:,axis);
end
for i=1:n
    Ai = A(:,i);
    T = twist2ht(Ai,-q(i)) * Mrelative(:,:,i);
    V(:,i+1) = Ai*qd(i) + adjoint(T)*V(:,i);
    Vdot(:,i+1) = Ai*qdd(i) + ad(V(:,i+1))*Ai*qd(i) + adjoint(T)*Vdot(:,i);
end
tau = zeros(n,1);
Tn = inv(params.M(:,:,n+1));
Fi = adjoint(Tn)'*params.Ftip + params.G(:,:,n)*Vdot(:,n+1) - ad(V(:,n+1))'*params.G(:,:,n)*V(:,n+1);
tau(n) = Fi'*A(:,n);
for i=n-1:-1:1
    Ti = twist2ht(A(:,i+1),-q(i+1)) * Mrelative(:,:,i+1);
    Fi = adjoint(Ti)'*Fi + params.G(:,:,i)*Vdot(:,i+1) - ad(V(:,i+1))'*params.G(:,:,i)*V(:,i+1);
    tau(i) = Fi' * A(:,i);
end
function AdT = adjoint(T)
    R = T(1:3,1:3);
    p = T(1:3,4);
    p1 = p(1);
    p2 = p(2);
    p3 = p(3);
    skewP = [0 -p3 p2; p3 0 -p1; -p2 p1 0];
    AdT = zeros(6,6);
    AdT(1:3,1:3) = R;
    AdT(4:6,4:6) = R;
    AdT(1:3,4:6) = 0; % explicitly denote because reader expects it
    AdT(4:6,1:3) = skewP * R;
end
function V_b = twistspace2body(V_s,T)
    R = T(1:3,1:3);
    p = T(1:3,4);
    p1 = p(1);
    p2 = p(2);
    p3 = p(3);
    skewP = [0 -p3 p2; p3 0 -p1; -p2 p1 0];
    adjTtransp = zeros(6,6);
    adjTtransp(1:3,1:3) = R';
    adjTtransp(4:6,4:6) = R';
    adjTtransp(1:3,4:6) = 0;
    adjTtransp(4:6,1:3) = -R' * skewP;
    V_b = adjTtransp * V_s;
end
function R = axisangle2rot(omega,theta)
    st = sin(theta);
    ct = cos(theta);
    w1 = omega(1);
    w2 = omega(2);
    w3 = omega(3);
    skewOmega = [0 -w3 w2; w3 0 -w1; -w2 w1 0];
    R = eye(3) + st*skewOmega + (1 - ct)*skewOmega*skewOmega;
end
function T = twist2ht(S,theta)
    T = zeros(4,4);
    T(4,4) = 1;
    omega = S(1:3);
    v = S(4:6);
    R = axisangle2rot(omega,theta);
    st = sin(theta);
    ct = cos(theta);
    w1 = omega(1);
    w2 = omega(2);
    w3 = omega(3);
    skewOmega = [0 -w3 w2; w3 0 -w1; -w2 w1 0];
    p = (eye(3)*theta + (1-ct)*skewOmega + (theta - st)*skewOmega*skewOmega) * v;
    T(1:3,1:3) = R;
    T(1:3,4) = p;
end
function adV = ad(V)
    w = V(1:3);
    v = V(4:6);
    adV = zeros(6,6);
    adV(1:3,1:3) = skew(w);
    adV(4:6,4:6) = skew(w);
    adV(1:3,4:6) = 0;
    adV(4:6,1:3) = skew(v);
end
function skewP = skew(p)
p1 = p(1);
p2 = p(2);
p3 = p(3);
skewP = [0 -p3 p2; p3 0 -p1; -p2 p1 0];
end
end
    n = size(params.jointPos,1);

    switch term
        case 'centripetal-coriolis'
            params.jointAcc = zeros(n,1);
            params.g = zeros(3,1);
            params.Ftip = zeros(6,1);
            h = rne(params);
        case 'gravity'
            params.jointAcc = zeros(n,1);
            params.jointVel = zeros(n,1);
            params.Ftip = zeros(6,1);
            h = rne(params);
        case 'ext-force'
            % Zero gravity and motion to obtain effect of external tip force only
            params.g = zeros(3,1);
            params.jointAcc = zeros(n,1);
            params.jointVel = zeros(n,1);
            h = rne(params);
        otherwise 
            error('I genuinely have no idea what you are trying to do.');      
    end
end
