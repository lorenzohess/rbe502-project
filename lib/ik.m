function q = ik(targetPt, currentQ, S, M)
% IK using analytic Jacobian.
% 
% Inputs:
%   targetPt: row vec
%   currentQ: row vec

    currentT = fkine(S, M, currentQ, "space"); 
    targetPt;
    currentP = currentT(1:3,4)';

    % The control loop: iterate over each target pose and use gradient descent
    % to solve joint angles (currentQ) for each point.
    % The control loop. The exit condition is when our robot is within the 1e-3
    % tolerance of target pose.
    while norm(targetPt - currentP) > 1e-3
        positionError = targetPt - currentP; % row
        
        % Compute analytic jacobian to find deltaQ.
        Ja = jacobaspace(S, M, currentQ); % jacobaspace wants row Q
        % DeltaQ is col here. transpose error for Ja' math equation
        deltaQ = 0.6 * (Ja' * positionError');
        currentQ = currentQ + deltaQ';
        
        currentT = fkine(S, M, currentQ, "space"); 
        currentP = currentT(1:3,4)'; % update current pose
    end
    q = currentQ;
end
