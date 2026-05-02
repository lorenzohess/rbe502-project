%% Generate desired joint trajectory from cartesian waypoints
clc, close all;
addpath("/home/lh/nextcloud-sync/wpi/rbe502-control/OpenManipulator-X")
addpath("../lib")

[S,M] = make_kinematics_model();
nJoints = 4;                     % degrees of freedom
robot   = make_robot();
homeQ   = zeros(1,nJoints);

t_sample = 0.01;                 % must match the control loop
T_seg    = 0.1;                  % seconds between consecutive circle waypoints
T_home   = 2.0;                  % seconds to move from home to first waypoint

%% Define the cartesian path: circle in the x-z plane at fixed y
nPathPts = 50;                  % number of points around the circle
y_plane  = 0.20;                 % circle lies in plane y = 0.20 m
radius   = 0.08;                 % circle radius [m] (diameter = 0.16 m)
xc       = 0.20;                 % circle center x
zc       = 0.20;                 % circle center z

% Parameterise the circle. linspace(0, 2*pi, nPathPts) places the final
% sample at theta = 2*pi, so the last point coincides with the first and
% the path closes naturally (mirroring the closed-square behaviour).
theta = linspace(0, 2*pi, nPathPts);
x = xc + radius * cos(theta);
z = zc + radius * sin(theta);
y = y_plane * ones(1, nPathPts);
path = [x; y; z];
fprintf('Path defined (%d points, closed circle r=%.2f m at y=%.2f m).\n', ...
        nPathPts, radius, y_plane);

%% Inverse kinematics at each waypoint
fprintf('Calculating the Inverse Kinematics... ');
jointValuesPerPathPoints = zeros(nJoints,nPathPts);
currentQ = homeQ;
for pathPtIdx = 1 : nPathPts
    targetPt = path(:, pathPtIdx)';
    jointValuesPerPathPoints(:,pathPtIdx) = ik(targetPt, currentQ, S, M);
    currentQ = jointValuesPerPathPoints(:, pathPtIdx)';
end
waypoints = jointValuesPerPathPoints;   % circle waypoints only (excludes home)
fprintf('IK done.\n');

%% Prepend home as an extra waypoint so the first segment is home -> first
%% circle point. This way the existing per-segment loop handles everything.
allWaypoints = [homeQ', jointValuesPerPathPoints];
nSegs        = size(allWaypoints, 2) - 1;
segTimes     = [T_home, repmat(T_seg, 1, nSegs - 1)];

%% Quintic interpolation between consecutive waypoints
fprintf('Generating quintic trajectory... ');
dt = t_sample;
q_desired      = [];
q_desired_dot  = [];
q_desired_ddot = [];
nbytes = 0;
for segIdx = 1 : nSegs
    fprintf(repmat('\b',1,nbytes));
    nbytes = fprintf('%3.0f%%', 100*(segIdx/nSegs));

    Tk    = segTimes(segIdx);
    t_seg = 0 : dt : Tk;
    jointPos_desired = zeros(nJoints,length(t_seg));
    jointVel_desired = zeros(nJoints,length(t_seg));
    jointAcc_desired = zeros(nJoints,length(t_seg));

    for jointIdx = 1 : nJoints
        params_traj.t         = [0 Tk];
        params_traj.time_step = dt;
        params_traj.q = [allWaypoints(jointIdx, segIdx) ...
                         allWaypoints(jointIdx, segIdx+1)];
        params_traj.v = [0 0];
        params_traj.a = [0 0];
        traj = make_trajectory('quintic', params_traj);
        jointPos_desired(jointIdx,:) = traj.q;
        jointVel_desired(jointIdx,:) = traj.v;
        jointAcc_desired(jointIdx,:) = traj.a;
    end

    % Drop the first sample of every segment except the first so the
    % junction sample isn't duplicated.
    if segIdx == 1
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
fprintf('Saved desired_trajectory.mat - %.2f s, %d samples.\n', ...
        t_traj(end), length(t_traj));