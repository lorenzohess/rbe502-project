function J_a = jacoba(B,M,q)    
% Compute analytic Jacobian Ja, such that pdot = Ja * qdot.
% Where pdot is EE linear velocities in the SPACE FRAME.
% B are screw axes in body frame since we're required to use
% screw axes in body.
    pose = fkine(B,M,q,'body');
    Jb = jacobe(B,M,q);

    % Convert body to space
    Js = adjoint(pose) * Jb;

    % Compute J_a from Js
    Jw = Js(1:3,:);
    Jv = Js(4:6,:);
    p = pose(1:3,4);
    p1 = p(1);
    p2 = p(2);
    p3 = p(3);
    skewP = [0 -p3 p2; p3 0 -p1; -p2 p1 0];
    J_a = Jv - skewP * Jw;
end
