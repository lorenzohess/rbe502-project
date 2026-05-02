function [S,M] = make_kinematics_model()
% MAKE_KINEMATICS_MODEL Calculates the Screw Axes and Home Configuration of
% a robot.
%
% Inputs: None
%
% Output: S - 4xn matrix whose columns are the screw axes of the robot
%         M - homogeneous transformation representing the home configuration

d1 = 0.077; % Length of Link 1 [m]
a2 = 0.128; % Length of Link 2 [m]
a2short = 0.024;
a3 = 0.124; % Length of Link 3 [m]
a4 = 0.126; % Length of Link 4 [m]

% space frame basis
x0 = [1 0 0];
y0 = [0 1 0];
z0 = [0 0 1];

% omega
w1 = z0;
w2 = -x0;
w3 = -x0;
w4 = -x0;

% p
p1 = [0 0 0];
p2 = [0 0 d1];
p3 = p2 + [0 a2short a2];
p4 = p3 + [0 a3 0];
pF = p4 + [0 a4 0];

%% YOUR CODE HERE
S = [w1 -cross(w1, p1);
     w2 -cross(w2, p2);
     w3 -cross(w3, p3);
     w4 -cross(w4, p4)]';

R = eye(3);
p = pF';
M = [R p; 0 0 0 1];
end
