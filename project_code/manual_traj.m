%% Teach-mode recording: move the arm by hand, log q / qdot / qddot
clc, clear all, close all;
addpath("../Communication_Code");
addpath("../generated_dynamics")
addpath("/home/lh/nextcloud-sync/wpi/rbe502-control/OpenManipulator-X")

%% Conversions
factor_degre_to_rad = pi/180;
factor_A_to_mA      = 1000;

%% Recording parameters
t_sample = 0.05;          % 20 Hz logging period
tfin     = 10;            % total recording time [s] - bump up if you need more
t        = 0:t_sample:tfin;
N        = length(t);
alpha    = 0.3;           % EMA factor: qdot_filt = alpha*qdot + (1-alpha)*qdot_prev_filt

%% Dynamic params for gravity / dynamic compensation while teaching
R = load('../Identification/identification_result.mat');
p = [R.p(1:6); ...
     R.x_opt_vec(1); R.x_opt_vec(2); R.x_opt_vec(3); R.x_opt_vec(4); ...
     R.x_opt_vec(5); R.x_opt_vec(6); R.x_opt_vec(7); ...
     R.x_opt_vec(8); R.x_opt_vec(9); R.x_opt_vec(10); ...
     R.x_opt_vec(11); R.x_opt_vec(12); R.x_opt_vec(13); ...
     R.x_opt_vec(14); R.x_opt_vec(15); R.x_opt_vec(16); ...
     R.id_info.g];

%% Robot init - current mode so motors are backdrivable
robot = Robot();
robot.writeMode('c');

%% Logs (4 x N)
q_log     = zeros(4, N);
qdot_log  = zeros(4, N);
qddot_log = zeros(4, N);

%% Initial reading and filter state
joint_readings = robot.getJointsReadings();
q_now          = (joint_readings(1,:) * factor_degre_to_rad)';
qdot_prev_filt = (joint_readings(2,:) * factor_degre_to_rad)';
q_log(:,1)     = q_now;
qdot_log(:,1)  = qdot_prev_filt;
% qddot_log(:,1) stays zero

disp("Move the arm now. Recording for " + tfin + " s...");

%% Loop
for k = 1:N
    tic

    joint_readings = robot.getJointsReadings();
    q_now    = (joint_readings(1,:) * factor_degre_to_rad)';
    qdot_raw = (joint_readings(2,:) * factor_degre_to_rad)';

    % EMA filter on qdot
    qdot_filt = alpha*qdot_raw + (1-alpha)*qdot_prev_filt;

    % qddot from filtered qdot via backward difference
    if k == 1
        qddot_now = zeros(4,1);
    else
        qddot_now = (qdot_filt - qdot_prev_filt) / t_sample;
    end

    q_log(:,k)     = q_now;
    qdot_log(:,k)  = qdot_filt;
    qddot_log(:,k) = qddot_now;

    % Gravity / dynamic compensation: hold the arm so the user can backdrive it.
    % Set q_des = q_now, qdot_des = qddot_des = 0, Kp = Kv = 0  -->  feed-forward only.
    tau = tau_computed_torque(q_now, qdot_filt, ...
                              q_now, zeros(4,1), zeros(4,1), ...
                              zeros(4), zeros(4), p);
    current_mA = torque_to_current(tau') * factor_A_to_mA;
    robot.writeCurrents(current_mA);

    qdot_prev_filt = qdot_filt;

    while toc < t_sample
    end
end

%% Stop arm
robot.writeCurrents([0 0 0 0]);
disp("Recording complete.");

%% Save in the format the sim loop expects
q_desired      = q_log;
q_desired_dot  = qdot_log;
q_desired_ddot = qddot_log;
t_traj         = t;
waypoints      = q_log;     % keep field for compatibility
path           = [];        % no cartesian path defined here

save('desired_trajectory.mat', ...
     'q_desired', 'q_desired_dot', 'q_desired_ddot', ...
     't_traj', 't_sample', 'waypoints', 'path');
fprintf("Saved desired_trajectory.mat - %.2f s, %d samples.\n", t_traj(end), N);
