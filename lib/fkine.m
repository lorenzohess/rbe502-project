function T = fkine(S,M,q,frame)
    % FKINE - compute forward kinematics of manipulator using screw theory.
    %
    % Input arguments:
    % S - the screw axes of the manipulator
    % M - the home configuration of the manipulator
    % q - the current joint values of the manipulator
    % frame - 'body' or 'space'
    %
    % Output arguments:
    % T - HT representing the pose of the link driven by the last joint in q

    % In body and space frame, the initial T is by definition the home configuration
    %
    % Example usage:
    %
    % The code below defines the screw axes of a 3DOF robot, its configuration q, and the home pose M
    % S = [0 0 1 0 0 0;
    % 1 0 0 0 0.3 0;
    % 0 0 0 0 0 -1]';
    % 
    % q = [rand() * 2 * pi, rand() * 2 * pi, rand() * 0.3];
    % 
    % R_home = [0 1 0; 1 0 0; 0 0 -1]';
    % t_home = [0.3 0 0.3]';
    % M = [R_home t_home; 0 0 0 1];
    % 
    % % Calculate the forward kinematics using the PoE in the space frame
    % fkine(S,M,q,'space')
    % 
    % % Calculate the forward kinematics using the PoE in the body frame
    % fkine(S,M,q,'body')

    T = M;
    
    if (frame == "space")
        % Product of Exponentials formula. Pre-multiply T by Ti, starting with
        % pre-multiplying by Ti for i=n-1. The last pre-multiplication is for i=1.
        % To implement pre-multiplication, count down from last axis to first axis.
        for axis = size(S,2):-1:1
            Ti = twist2ht(S(1:6,axis), q(axis));
            T = Ti * T;
        end
    else % body frame
        % Product of Exponentials formula. Post-multiply T by Ti, starting with
        % post-multiplying by Ti for i=1. The last pre-multiplication is for i=n.
        % To implement post-multiplication, count up from first axis to last axis.
        for axis = 1:size(S,2)
            screwAxisSpace = S(1:6,axis);
            TiBody = twist2ht(screwAxisSpace, q(axis));
            T = T * TiBody;
        end
    end
end
