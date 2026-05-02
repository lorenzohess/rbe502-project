function judge = NearZero(near)
    % NEARZERO - Return true if Euclidean norm of input vector is < 1e-6.
    % Input arguments:
    % near - the input vector
    %
    % Output arguments:
    % judge - the boolean result

judge = norm(near) < 1e-6;
end
