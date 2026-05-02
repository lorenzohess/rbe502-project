function J = jacob0(S,q)
    % JACOB0 - Construct Jacobian associated with screw axes and joint variables.
    %
    % Input arguments:
    % S - list of screw axes
    % q - list of corresponding joint variables
    %
    % Output arguments:
    % J - the Jacobian

    % J col vecs are the adjoint trs. of each screw axis with the HT up to,
    % but not including, that screw axis.

    % Set up containers for twists and HTs.
    Vs = cell(1,6);
    Ts = cell(1,6);

    % Compute twists and HTs for each axis, index i.
    for i = 1:size(S,2)
        Vi = S(1:6,i);
        Vs{i} = Vi;
        Ts{i} = twist2ht(Vi,q(i));
    end

    % Compute J using adjoint transformation of twist and HT.
    J = []; % Container for J
    % Iterate through each axis, index i.
    for i = 1:size(S,2)
        % For each axis, we must compute the product of T1 * T2 * ... Ti-1.
        % Store T1 as T
        T = Ts{1};
        % Then pre-multiply by T2 up to Ti-1
        for j = 2:i % 
            T = T * Ts{j};
        end
        % The J for this axis, i, is adjoint tr. of the twist and the HT.
        Ji = adjoint(T) * Vs{i};
        J = [J Ji];
    end
end
