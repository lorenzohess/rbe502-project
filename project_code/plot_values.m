%% plot_values.m
% Expects: t, q_desired (4xN), q_real (4x(N+1)), q_desired_dot (4xN),
%          q_dot_real (4x(N+1)), tau_k (4xN)

N = length(t);

%% Joint positions (desired vs actual)
figure('Name', 'Joint Positions')
for i = 1:4
    subplot(2,2,i)
    plot(t, q_desired(i,:), 'r.')
    hold on
    plot(t, q_real(i,1:N), 'g.')
    xlabel('Time [s]')
    ylabel(['q', num2str(i), ' [rad]'])
    title(['Joint ', num2str(i), ' Angle'])
    legend('Desired', 'Current', 'Location', 'best')
    grid on
end

%% Tracking error norm per joint
figure('Name', 'Tracking Error')
e = q_desired - q_real(:,1:N);
for i = 1:4
    subplot(2,2,i)
    plot(t, abs(e(i,:)), 'g.')
    xlabel('Time [s]')
    ylabel(['|e_', num2str(i), '| [rad]'])
    title(['Joint ', num2str(i), ' Tracking Error'])
    grid on
end

%% Joint velocities (desired vs actual)
figure('Name', 'Joint Velocities')
for i = 1:4
    subplot(2,2,i)
    plot(t, q_desired_dot(i,:), 'r.')
    hold on
    plot(t, q_dot_real(i,1:N), 'g.')
    xlabel('Time [s]')
    ylabel(['dq', num2str(i), ' [rad/s]'])
    title(['Joint ', num2str(i), ' Velocity'])
    legend('Desired', 'Current', 'Location', 'best')
    grid on
end

%% Joint torques
figure('Name', 'Joint Torques')
for i = 1:4
    subplot(2,2,i)
    plot(t, tau_k(i,:), 'g.')
    xlabel('Time [s]')
    ylabel(['tau', num2str(i), ' [Nm]'])
    title(['Joint ', num2str(i), ' Control Torque'])
    grid on
end
