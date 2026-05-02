function Tinv = TransInv(T)
    [R, p] = TransToRp(T);
    Tinv = [R' -R'*p;
            0 0 0 1];
end
