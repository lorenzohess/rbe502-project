%% Setup robot
% computed_torque_on_arm_to_home
clc, clear all, close all;
n=4;

%% Add Subfolder
addpath("../Communication_Code");
addpath("../generated_dynamics")
addpath("/home/lh/nextcloud-sync/wpi/rbe502-control/OpenManipulator-X")

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

Kp = diag([20 30 50 40]);
Kv = diag([4 6 6 2]);
A = [zeros(n) eye(n); -Kp -Kv];
Q = eye(2*n);
P = lyap(A', Q);
rho = 2;

%% Constant trajectory
% t_sample       = 0.04;
% tfin           = 10;
% t = 0:t_sample:tfin;
% q1_desired = 0.0*ones(1, length(t));
% q2_desired = 0.0*ones(1, length(t));
% q3_desired = 0.0*ones(1, length(t));
% q4_desired = 1.0*ones(1, length(t));
% q_desired = [q1_desired; q2_desired; q3_desired; q4_desired];
% q_desired_dot = [0*q1_desired; 0*q2_desired; 0*q3_desired; 0*q4_desired];
% q_desired_ddot = [0*q1_desired; 0*q2_desired; 0*q3_desired; 0*q4_desired];

%% Load desired trajectory from file 
%square_trajectory
traj_data      = load('desired_trajectory.mat');
q_desired      = traj_data.q_desired;       % 4 x N
q_desired_dot  = traj_data.q_desired_dot;   % 4 x N
q_desired_ddot = traj_data.q_desired_ddot;  % 4 x N
t_sample       = traj_data.t_sample;
tfin           = (size(q_desired,2)-1) * t_sample;
t = 0:t_sample:tfin;

%% Define robot
robot = Robot();

%% Define the type of low level control of the robot this is current mode
robot.writeMode('c');

%% Joint Positions
q_real = zeros(4, length(t)+1);
q_dot_real = zeros(4, length(t)+1);
current_real = zeros(4, length(t)+1);

%% Read Initial Conditions
joint_readings = robot.getJointsReadings();
q_real(:, 1) = joint_readings(1, :)*factor_degre_to_rad;
q_dot_real(:, 1) = joint_readings(2, :)*factor_degre_to_rad;
current_real(:, 1) = joint_readings(3, :)*factor_mA_to_A;
%% Control Loop
for k = 1:length(t) 
    tic
    %% Create Control Law Your Controller goes Here
    q_now = (joint_readings(1, :)*factor_degre_to_rad)';
    q_dot_now = (joint_readings(2, :)*factor_degre_to_rad)';
    tau_k(:, k) = tau_robust(q_now, q_dot_now, q_desired(:,k), q_desired_dot(:,k), q_desired_ddot(:,k), Kp, Kv, P, rho, p_bar);

    torques = [tau_k(1, k), tau_k(2, k), tau_k(3, k), tau_k(4, k)];
    %% This is the mapping to Amperes
    current = torque_to_current(torques);

    %% This is the mapping to mA
    current_mA = current*factor_A_to_mA;

    robot.writeCurrents(current_mA);

    %% Sample time
    dt(k) = toc;

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
%% Animate the robot and overlay desired/actual end-effector paths
% figure('Name','Robot animation','NumberTitle','off')
% robot.plot(q_real(:,1:length(t))', 'trail', 'b-', 'fps', 30)
% hold on
% plot3(p_desired_log(1,:), p_desired_log(2,:), p_desired_log(3,:), 'r-', 'LineWidth', 1.5)
% legend('','Desired','Location','best')

%% Static 3D comparison plot
% figure('Name','EE trajectory','NumberTitle','off')
% plot3(p_desired_log(1,:), p_desired_log(2,:), p_desired_log(3,:), 'r-', 'LineWidth', 1.5); hold on
% plot3(p_real_log(1,:),    p_real_log(2,:),    p_real_log(3,:),    'b-', 'LineWidth', 1.5)
% xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]')
% grid on; axis equal
% legend('Desired','Actual'); title('End-effector trajectory')

% q_desired = [q1_desired; q2_desired; q3_desired; q4_desired];
% tau_all = tau_k;

figure(1)
for i = 1:4
    subplot(2,2,i)
    plot(t, q_desired(i,:), 'r.')
    hold on
    plot(t, q_real(i,1:length(t)), 'b.')
    xlabel('Time [s]')
    ylabel(['q', num2str(i), ' [rad]'])
    title(['Joint ', num2str(i), ' Angle'])
    legend('Desired', 'Current', 'Location', 'best')
    grid on
end

% figure(2)
% for i = 1:4
%     subplot(2,2,i)
%     plot(t, q_dot_real(i,1:length(t)), 'k.')
%     xlabel('Time [s]')
%     ylabel(['dq', num2str(i), ' [rad/s]'])
%     title(['Joint ', num2str(i), ' Velocity'])
%     grid on
% end

% figure(3)
% for i = 1:4
%     subplot(2,2,i)
%     plot(t, tau_all(i,:), 'm.')
%     xlabel('Time [s]')
%     ylabel(['tau', num2str(i), ' [Nm]'])
%     title(['Joint ', num2str(i), ' Control Torque'])
%     grid on
% end
