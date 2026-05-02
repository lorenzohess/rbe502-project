%% Generate desired joint trajectory from cartesian waypoints
clc, close all;
addpath("/home/lh/nextcloud-sync/wpi/rbe502-control/OpenManipulator-X")
addpath("../lib")
[S,M] = make_kinematics_model();
nJoints = 4; % degrees of freedom
robot = make_robot();
homeQ = zeros(1,nJoints);

t_sample = 0.05;   % must match the control loop
T_seg    = 0.1;    % seconds spent moving between consecutive waypoints

%% Define the cartesian path: square in the y–z plane at fixed x
nPathPts = 1000;                % total points around the square
x_plane  = 0.20;                % square lies in the plane x = 0.20 m
side     = 0.16;                % side length [m]
yc       = 0.0;                 % square center y
zc       = 0.20;                % square center z

% Four corners, traversed counter-clockwise as seen from +x looking back
half = side/2;
corners = [yc-half, zc-half;    % bottom-left
           yc+half, zc-half;    % bottom-right
           yc+half, zc+half;    % top-right
           yc-half, zc+half;    % top-left
           yc-half, zc-half];   % back to start

% Distribute points evenly along the perimeter
ptsPerEdge = round(nPathPts/4);
y = []; z = [];
for k = 1:4
    y = [y, linspace(corners(k,1), corners(k+1,1), ptsPerEdge+1)];
    z = [z, linspace(corners(k,2), corners(k+1,2), ptsPerEdge+1)];
    % drop the last point so it isn't duplicated by the next edge's start
    y(end) = []; z(end) = [];
end
x = x_plane * ones(1, length(y));
nPathPts = length(y);           % update in case rounding changed it

path = [x; y; z];
fprintf('Path defined (%d points, square %.2f m on a side at x=%.2f m).\n', ...
        nPathPts, side, x_plane);

%% Inverse kinematics at each waypoint
fprintf('Calculating the Inverse Kinematics... ');
jointValuesPerPathPoints = zeros(nJoints,nPathPts);
currentQ = homeQ; % row vec
for pathPtIdx = 1 : nPathPts
    targetPt = path(:, pathPtIdx)';
    jointValuesPerPathPoints(:,pathPtIdx) = ik(targetPt, currentQ, S, M);
    currentQ = jointValuesPerPathPoints(:, pathPtIdx)';
end
waypoints = jointValuesPerPathPoints;
fprintf('IK done.\n');

%% Quintic interpolation between consecutive waypoints
fprintf('Generating quintic trajectory... ');
dt = t_sample;
q_desired      = [];
q_desired_dot  = [];
q_desired_ddot = [];

nbytes = 0;
for pathPtIdx = 1 : nPathPts - 1
    fprintf(repmat('\b',1,nbytes));
    nbytes = fprintf('%3.0f%%', 100*(pathPtIdx/(nPathPts - 1)));

    t_seg = 0 : dt : T_seg;
    jointPos_desired = zeros(nJoints,length(t_seg));
    jointVel_desired = zeros(nJoints,length(t_seg));
    jointAcc_desired = zeros(nJoints,length(t_seg));

    for jointIdx = 1 : nJoints
        params_traj.t         = [0 T_seg];
        params_traj.time_step = dt;
        params_traj.q = [jointValuesPerPathPoints(jointIdx,pathPtIdx) ...
                         jointValuesPerPathPoints(jointIdx,pathPtIdx+1)];
        params_traj.v = [0 0];
        params_traj.a = [0 0];
        traj = make_trajectory('quintic', params_traj);
        jointPos_desired(jointIdx,:) = traj.q;
        jointVel_desired(jointIdx,:) = traj.v;
        jointAcc_desired(jointIdx,:) = traj.a;
    end

    % Concatenate; drop the first sample of subsequent segments so the
    % junction point is not duplicated.
    if pathPtIdx == 1
        q_desired      = [q_desired,      jointPos_desired];
        q_desired_dot  = [q_desired_dot,  jointVel_desired];
        q_desired_ddot = [q_desired_ddot, jointAcc_desired];
    else
        q_desired      = [q_desired,      jointPos_desired(:,2:end)];
        q_desired_dot  = [q_desired_dot,  jointVel_desired(:,2:end)];
        q_desired_ddot = [q_desired_ddot, jointAcc_desired(:,2:end)];
    end
end
fprintf('\nTrajectory done.\n');

%% Save
t_traj = (0:size(q_desired,2)-1) * dt;
save('desired_trajectory.mat', ...
     'q_desired', 'q_desired_dot', 'q_desired_ddot', ...
     't_traj', 't_sample', 'waypoints', 'path');
fprintf('Saved desired_trajectory.mat — %.2f s, %d samples.\n', ...
        t_traj(end), length(t_traj));
