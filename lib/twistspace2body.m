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
