function J_a = jacobaspace(S, M, q)
% jacoba  Compute the analytic Jacobian mapping joint velocities to EE linear velocity in the space frame.
%
% Inputs:
%   S  (double [6xn]) - screw axes in the space frame
%   M  (double [4x4]) - home configuration of the EE
%   q  (double [1xn]) - joint variables
%
% Output:
%   J_a (double [3xn]) - analytic Jacobian
    pose = fkine(S, M, q, 'space');
    Js = jacob0(S, q);
    Jw = Js(1:3,:);
    Jv = Js(4:6,:);
    p = pose(1:3,4);
    skewP = [0 -p(3) p(2); p(3) 0 -p(1); -p(2) p(1) 0];
    J_a = Jv - skewP * Jw;
end
