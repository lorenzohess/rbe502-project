function jointAcc = fdyn(params)
%% FDYN Implements the Forward Dynamics of a Serial Kinematic Chain
%
 % Inputs: params - a structure containing the following fields:
%           params.g - 3-dimensional column vector describing the acceleration of gravity
%           params.S - 6xn matrix of screw axes (each column is an axis)
%           params.M - 4x4xn home configuration matrix for each link
%           params.G - 6x6xn spatial inertia matrix for each link
%           params.jointPos - n-dimensional column vector of joint coordinates
%           params.jointVel - n-dimensional column vector of joint velocities
%           params.tau - n-dimensional column vector of joint torques/forces
%           params.Ftip - 6-dimensional column vector representing the wrench applied at the tip
%
% Output:   jointAcc - n-dimensional column vector of joint accelerations
%
% Author: L. Fichera, loris@wpi.edu
% Last Updated: 4/02/2026

% Compute mass matrix for current joint configuration
MM = MassMatrixCalculator(params.jointPos, params.S, params.M, params.G);

% Compute Coriolis and centripetal torque vector
cc = calcdynterms(params, 'centripetal-coriolis');

% Compute gravity torque vector
gg = calcdynterms(params, 'gravity');

% Map external tip force into joint torques
JtFtip = calcdynterms(params, 'ext-force');

% Solve for joint accelerations
jointAcc = MM \ (params.tau - cc - gg - JtFtip);

end

