%% Setup robot
% adaptive_computed_torque_on_arm
clc, clear all, close all;
n=4;

%% Add Subfolder
addpath("../Communication_Code");
addpath("../generated_dynamics");
addpath("/home/lh/nextcloud-sync/wpi/rbe502-control/OpenManipulator-X");

%% Factor from degres to rad
factor_degre_to_rad = pi/180;
factor_mA_to_A = 1/1000;
factor_A_to_mA = 1000/1;

%% System parameters
R =load('../Identification/identification_result.mat');
p = [R.p(1:6); ...
     R.x_opt_vec(1); R.x_opt_vec(2); R.x_opt_vec(3); R.x_opt_vec(4); ...
     R.x_opt_vec(5); R.x_opt_vec(6); R.x_opt_vec(7); ...
     R.x_opt_vec(8); R.x_opt_vec(9); R.x_opt_vec(10); ...
     R.x_opt_vec(11); R.x_opt_vec(12); R.x_opt_vec(13); ...
     R.x_opt_vec(14); R.x_opt_vec(15); R.x_opt_vec(16); ...
     R.id_info.g];
pf = [R.x_opt_vec(17); R.x_opt_vec(18); R.x_opt_vec(19); R.x_opt_vec(20)];
p_bar = p;

% %% Damping analysis for current Kp, Kv
% kp_vec   = diag(Kp);
% kv_vec   = diag(Kv);
% wn_vec   = sqrt(kp_vec);
% zeta_vec = kv_vec ./ (2*wn_vec);
% labels = repmat("underdamped", n, 1);
% labels(zeta_vec >= 0.9 & zeta_vec <= 1.1) = "critical";
% labels(zeta_vec >  1.1) = "overdamped";
% T = table((1:n)', kp_vec, kv_vec, wn_vec, zeta_vec, labels, ...
%           'VariableNames', {'Joint','Kp','Kv','omega_n','zeta','Behavior'});
% disp(T)

%% Adaptation setup
idx_lin = 7:22;
pi_hat  = p_bar(idx_lin);   % 16x1: [m1..m4, Ixx1..Izz4]
n_p    = length(pi_hat);
R_gain = construct_R;
R_inv = pinv(R_gain);
alpha_acc = 0.2;                 % low-pass on finite-diff qddot

%% Constant trajectory
% Kp = diag([0.15 0.15 0.5 0.25]);
% Kv = diag([0.1 0.08 0.075 0.05]);
% t_sample       = 0.04;
% tfin           = 15;
% t = 0:t_sample:tfin;
% q1_desired = deg2rad(39)*ones(1, length(t));
% q2_desired = deg2rad(-28)*ones(1, length(t));
% q3_desired = deg2rad(55)*ones(1, length(t));
% q4_desired = deg2rad(-57)*ones(1, length(t));
% q_desired = [q1_desired; q2_desired; q3_desired; q4_desired];
% q_desired_dot = [0*q1_desired; 0*q2_desired; 0*q3_desired; 0*q4_desired];
% q_desired_ddot = [0*q1_desired; 0*q2_desired; 0*q3_desired; 0*q4_desired];


%% Load desired trajectory from file 
%square_trajectory
% traj_data      = load('desired_trajectory_plus.mat');
% q_desired      = traj_data.q_desired;       % 4 x N
% q_desired_dot  = traj_data.q_desired_dot;   % 4 x N
% q_desired_ddot = traj_data.q_desired_ddot;  % 4 x N
% t_sample       = traj_data.t_sample;
% tfin           = (size(q_desired,2)-1) * t_sample;
% t = 0:t_sample:tfin;
% Kp = diag([0.2 0.3 0.3 0.2]);
% Kv = diag([0.05 0.05 0.05 0.05]);

%% Sinusoidal
Kp = diag([0.2 0.35 0.6 0.2]);
Kv = diag([0.075 0.05 0.05 0.05]);
t_sample = 0.04;
tfin = 10;
t = 0:t_sample:tfin;

omega = 1.0;  % angular frequency (rad/s). Feel free to change (e.g. 0.5 for slower motion, 2.0 for faster)

q1_desired = 0.5 * sin(omega * t);
q2_desired = -0.5 * sin(omega * t);
q3_desired = -0.5 * sin(omega * t);          % sinusoidal position \pm 0.5 rad
q4_desired = -0.5 * sin(omega * t);

q_desired = [q1_desired; q2_desired; q3_desired; q4_desired];

% Velocity (automatically sinusoidal)
q1_desired_dot = 0.5 * omega * cos(omega * t);
q2_desired_dot = -0.5 * omega * cos(omega * t);
q3_desired_dot = -0.5 * omega * cos(omega * t);   % velocity = d(q3)/dt
q4_desired_dot = -0.5 * omega * cos(omega * t);

q_desired_dot = [q1_desired_dot; q2_desired_dot; q3_desired_dot; q4_desired_dot];

% Acceleration (automatically sinusoidal)
q1_desired_ddot = -0.5 * omega^2 * sin(omega * t);
q2_desired_ddot = 0.5 * omega^2 * sin(omega * t);
q3_desired_ddot = 0.5 * omega^2 * sin(omega * t);  % acceleration = d²(q3)/dt²
q4_desired_ddot = 0.5 * omega^2 * sin(omega * t);

q_desired_ddot = [q1_desired_ddot; q2_desired_ddot; q3_desired_ddot; q4_desired_ddot];

Mbar = M_fun(q_desired(:,1), p_bar);
A = [zeros(n) eye(n); -pinv(Mbar)*Kp -pinv(Mbar)*Kv];
Q = eye(2*n);
P = lyap(A', Q);

%% Define robot
robot = Robot();

%% Define the type of low level control of the robot this is current mode
robot.writeMode('c');

%% Joint Positions
q_real = zeros(4, length(t)+1);
q_dot_real = zeros(4, length(t)+1);
current_real = zeros(4, length(t)+1);
q_ddot_est = zeros(4, length(t));
pi_hat_log = zeros(n_p, length(t)+1);
pi_hat_log(:,1) = pi_hat;

%% Read Initial Conditions
joint_readings = robot.getJointsReadings();
q_real(:, 1) = joint_readings(1, :)*factor_degre_to_rad;
q_dot_real(:, 1) = joint_readings(2, :)*factor_degre_to_rad;
current_real(:, 1) = joint_readings(3, :)*factor_mA_to_A;
q_dot_prev = q_dot_real(:, 1);
%% Control Loop
for k = 1:length(t) 
    tic
    %% Create Control Law Your Controller goes Here
    q_now = (joint_readings(1, :)*factor_degre_to_rad)';
    q_dot_now = (joint_readings(2, :)*factor_degre_to_rad)';

    %% Estimate qddot for adaptation law (filtered finite-difference)
    if k == 1
        q_ddot_meas = q_desired_ddot(:,k);
    else
        q_ddot_meas = (q_dot_now - q_dot_prev) / dt(k-1);
    end
    [tau_k(:, k), pi_hat] = tau_adaptive(q_now, q_dot_now, ...
                                         q_desired(:,k), q_desired_dot(:,k), q_desired_ddot(:,k), ...
                                         Kp, Kv, P, R_inv, pi_hat, p_bar, t_sample);
    pi_hat_log(:, k+1) = pi_hat;

    torques = [tau_k(1, k), tau_k(2, k), tau_k(3, k), tau_k(4, k)];
    %% This is the mapping to Amperes
    current = torque_to_current(torques);

    %% This is the mapping to mA
    current_mA = current*factor_A_to_mA;
    % current_mA = clip(current_mA, 0, 1800)

    robot.writeCurrents(current_mA);

    %% Sample time
    dt(k) = toc;
    q_dot_prev = q_dot_now;

    %% Update measurements
    joint_readings = robot.getJointsReadings();
    q_real(:, k+1) = joint_readings(1, :)*factor_degre_to_rad;
    q_dot_real(:, k+1) = joint_readings(2, :)*factor_degre_to_rad;
    current_real(:, k+1) = joint_readings(3, :)*factor_mA_to_A;

    while toc < t_sample
    end
end

%% Final Values
tau = [0,0,0,0];
current = tau;
robot.writeCurrents(current); % Write joints to zero position
disp("Movement Complete")

%% Plotting
figure(1)
for i = 1:4
    subplot(2,2,i)
    plot(t, q_desired(i,:), 'r.')
    hold on
    plot(t, q_real(i,1:length(t)), 'g.')
    xlabel('Time [s]')
    ylabel(['q', num2str(i), ' [rad]'])
    title(['Joint ', num2str(i), ' Angle'])
    legend('Desired', 'Current', 'Location', 'best')
    grid on
end

% figure(2)
% plot_pihat
