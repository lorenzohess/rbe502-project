%% Normalized pi_hat plot (every param starts at 1.0)
figure('Name','pi_hat (normalized)')
N_log = length(t_full);
pi_norm = pi_hat_log(:, 1:N_log) ./ pi_init;   % 16 x N

subplot(2,1,1)
plot(t_full, pi_norm(1:4, :)')
ylabel('relative to initial')
legend(labels(1:4), 'Location','eastoutside')
title('Mass estimates (normalized)')
yline(1, 'k--'); grid on

subplot(2,1,2)
plot(t_full, pi_norm(5:16, :)')
ylabel('relative to initial')
xlabel('Time [s]')
legend(labels(5:16), 'Location','eastoutside')
title('Inertia estimates (normalized)')
yline(1, 'k--'); grid on
