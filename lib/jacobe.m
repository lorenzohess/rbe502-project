function J_b = jacobe(B,M,q)    
    % JACOBE - Construct body Jacobian from body-frame screw axes and joint variables in body i.e. EE frame.
    %
    % Input arguments:
    % B - list of screw axes in BODY FRAME
    % M - home configuration H.T.
    % q - list of corresponding joint variables
    %
    % Output arguments:
    % J_b - body Jacobian

    % J_b column i = Ad(e^{-[B_n]q_n} ... e^{-[B_{i+1}]q_{i+1}}) * B_i
    % J_b column n = B_n (no adjoint needed)
    n = size(B,2);
    J_b = zeros(6,n);

    % Compute each column of J_b
    for i = n-1:-1:1
        % Compute Adi. First, compute T from body to joint.
        % Initialize T to e^(-[Bn]*thetan)
        Bn = B(:,n);
        T = twist2ht(-Bn, q(n));

        % We will insert j = n after the loop, so iterate from j = n-1 down
        % to j = i+1, by definition
        for j = n-1:-1:i+1
            Bj = B(:,j);
            T = T * twist2ht(-Bj, q(j));
        end

        J_b(:,i) = adjoint(T) * B(:,i);
    end

    % Insert Bn
    Bn = B(:,n);
    J_b(:,n) = Bn;
end
