function [tau,V,Vdot] = rne(params)
%% RNE Implements the Recursive Newton-Euler Inverse Dynamics Algorithm
%
% Inputs: params - a structure containing the following fields:
%           params.g - 3-dimensional row vector describing the acceleration of gravity
%           params.S - 6xn matrix of screw axes (each column is an axis)
%           params.M - 4x4xn home configuration matrix for each link
%           params.G - 6x6xn spatial inertia matrix for each link
%           params.jointPos - n-dimensional column vector of joint coordinates
%           params.jointVel - n-dimensional column vector of joint velocities
%           params.jointAcc - n-dimensional column vector of joint accelerations
%           params.Ftip - 6-dimensional column vector representing the
%           wrench applied at the tip
%
% Output: tau  - n-dimensional column vector of generalized joint forces
%         V    - 6x(n+1) matrix - each column represents the twist of one of the robot's links
%         Vdot - 6x(n+1) matrix - each column represents the acceleration of one of the robot's links
%
% Forward iterations
%% Robot Definition:
n = size(params.S,2);    % Number of links in the kinematic chain

% Define the screw axes of each joint, expressed in the space frame
S = params.S;

% Initialize the twists and accelerations of each link
V = zeros(6,n+1);
Vdot = zeros(6,n+1);
% Initialize V0,Vdot0
V(:,1) = [0 0 0 0 0 0]';
Vdot(:,1) = [0; 0; 0; -params.g'];
 
% Initialize the joint positions and velocities
q = params.jointPos;
qd = params.jointVel;
qdd = params.jointAcc;

% Home configuration matrices of {i-1} in {i} when qi = 0.
Mrelative = zeros(4,4,n+1);
% Let pose of {j} in {i} be Mij, e.g. M21 = {1} in {2}.
% Let pose of {i} in {0} as Mi, e.g. M2 = params.M(:,:,2) = {2} in {0}.
% Then Mij = inv(Mi) * Mj,
%   e.g. i=1,j=2, {2} in {1} = M12 = inv(M1) * M2

% In FD loop we want M(i)(i-1) i.e. {i-1} in {i}, e.g. M10 = {0} in {1}.
% In FD loop we do frame i = 1:n, so Mrelative(:,:,i) should correspond
%   with M(i)(i-1).
% So when we build Mrelative, Mrelative(:,:,i) = Mij, where j=i-1.
%   Formula: Mrelative(:,:,i) = inv(Mi) * Mj.

% ..... just invert params.M, which turns out to be relative, not absolute.
for i=1:n
    Mrelative(:,:,i) = TransInv(params.M(:,:,i));
end

% Calculate the screw axes of each joint, expressed in the local link
% frame.
% First, find Mabs, i.e. {i} in {0}.
Mabs = zeros(4,4,n);
Mabs(:,:,1) = params.M(:,:,1); % M1 = M01
for i=2:n
    Mabs(:,:,i) = Mabs(:,:,i-1) * params.M(:,:,i);
end

A = zeros(6,n);
for axis = 1:n
    % Ai is screw axis of joint i in {i}
    A(:,axis) = adjoint(TransInv(Mabs(:,:,axis))) * S(:,axis);
end

% Forward iterations
for i=1:n
    Ai = A(:,i);
    T = twist2ht(Ai,-q(i)) * Mrelative(:,:,i);
    V(:,i+1) = Ai*qd(i) + adjoint(T)*V(:,i);
    Vdot(:,i+1) = Ai*qdd(i) + ad(V(:,i+1))*Ai*qd(i) + adjoint(T)*Vdot(:,i);
end
% Backward iterations
tau = zeros(n,1);

Tn = TransInv(params.M(:,:,n+1));
Fi = adjoint(Tn)'*params.Ftip + params.G(:,:,n)*Vdot(:,n+1) - ad(V(:,n+1))'*params.G(:,:,n)*V(:,n+1);
tau(n) = Fi'*A(:,n);

for i=n-1:-1:1
    Ti = twist2ht(A(:,i+1),-q(i+1)) * Mrelative(:,:,i+1);
    Fi = adjoint(Ti)'*Fi + params.G(:,:,i)*Vdot(:,i+1) - ad(V(:,i+1))'*params.G(:,:,i)*V(:,i+1);
    tau(i) = Fi' * A(:,i);
end
end
