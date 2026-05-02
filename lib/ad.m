function adV = ad(V)
    w = V(1:3);
    v = V(4:6);
    adV = zeros(6,6);
    adV(1:3,1:3) = skew(w);
    adV(4:6,4:6) = skew(w);
    adV(1:3,4:6) = 0;
    adV(4:6,1:3) = skew(v);
end
