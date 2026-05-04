%% plot_pihat.m
% Expects: t (1xN), pi_hat_log (16x(N+1))

N = length(t);
t_plot = [0, t];

labels = {'m_1','m_2','m_3','m_4', ...
          'I_{xx1}','I_{yy1}','I_{zz1}', ...
          'I_{xx2}','I_{yy2}','I_{zz2}', ...
          'I_{xx3}','I_{yy3}','I_{zz3}', ...
          'I_{xx4}','I_{yy4}','I_{zz4}'};

figure('Name', 'Adaptive Parameter Estimates')

subplot(2,1,1)
plot(t_plot, pi_hat_log(1:4, :)')
ylabel('\pi_{hat}')
legend(labels(1:4), 'Location', 'eastoutside')
title('Mass Estimates')
grid on

subplot(2,1,2)
plot(t_plot, pi_hat_log(5:16, :)')
ylabel('\pi_{hat}')
xlabel('Time [s]')
legend(labels(5:16), 'Location', 'eastoutside')
title('Inertia Estimates')
grid on
