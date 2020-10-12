fontSize=40;
legFontSize = 35;%31.5;
titleFontSize = 25;

marker_algo1_32 = 'bo-';
marker_algo1_8 = 'k^-';
marker_Delta_1 = 'rs-';
marker_Delta_0x2 = 'md-';
marker_Delta_0x15 = 'cp-';
markerSize = 15;

covariance_list = [0.05, 0.1, 0.2, 0.5];
%% The data below were generated separately and copied over
sreachtools_32_volume = [36.0000, 36.0000, 35.5512, 9.5328];
sreachtools_32_time = [9.4708, 9.5038, 9.2724, 9.1279];
sreachtools_8_volume = [36.0000, 36.0000, 34.6475, 8.6733];
sreachtools_8_time = [2.9932, 2.9516, 2.9918, 2.7555];
grid_sizes = [1, 0.2, 0.15];
stochy_volume = [36.         , 36.         , 36.        ;
                 16.         , 36.         , 36.        ;
                  0.         , 35.52       , 35.91418355;
                  0.         ,  6.88       , 11.2205006 ];
stochy_time = [6.16940e-01, 3.39577e+02, 1.28049e+03;
               4.88146e-01, 3.87457e+02, 1.35564e+03;
               5.05408e-01, 4.01314e+02, 1.38601e+03;
               5.01404e-01, 3.99440e+02, 1.41098e+03];

%% Volume
n_x_vals = 4;
figure(1);
clf;
set(gca,'FontSize',fontSize);
hold on;
plot(1:n_x_vals, stochy_volume(:, 1), marker_Delta_1, ...
    'MarkerSize', markerSize, 'LineWidth', 2, 'LineStyle', ':', ...
    'MarkerFaceColor', marker_Delta_1(1), ...
    'DisplayName', '\texttt{StocHy} (\texttt{IMDP}, $\Delta$=1)');
plot(1:n_x_vals, stochy_volume(:, 2), marker_Delta_0x2, ...
    'MarkerSize', markerSize, 'LineWidth', 2, 'LineStyle', ':', ...
    'MarkerFaceColor', marker_Delta_0x2(1), ...
    'DisplayName', '\texttt{StocHy} (\texttt{IMDP}, $\Delta$=0.2)');
plot(1:n_x_vals, stochy_volume(:, 3), marker_Delta_0x15, ...
    'MarkerSize', markerSize, 'LineWidth', 2, 'LineStyle', ':', ...
    'MarkerFaceColor', marker_Delta_0x15(1), ...
    'DisplayName', '\texttt{StocHy} (\texttt{IMDP}, $\Delta$=0.15)');
plot(1:n_x_vals, sreachtools_32_volume, marker_algo1_32, ...
    'MarkerSize', markerSize, 'LineWidth', 2, 'LineStyle', ':', ...
    'MarkerFaceColor', marker_algo1_32(1), ...
    'DisplayName', 'Alg. 1 (chance const., $|\mathcal{D}|=32$)');
plot(1:n_x_vals, sreachtools_8_volume, marker_algo1_8, ...
    'MarkerSize', markerSize, 'LineWidth', 2, 'LineStyle', ':', ...
    'MarkerFaceColor', marker_algo1_8(1), 'MarkerEdgeColor', 'k', ...
    'DisplayName', 'Alg. 1 (chance const., $|\mathcal{D}|=8$)');
xlabel('Variance ($\sigma^2$)','interpreter','latex');
ylabel('Stochastic reach set vol.','interpreter','latex');
set(gca, 'XTick', 1:n_x_vals);
set(gca, 'XTickLabels', covariance_list);
leg = legend('Location', 'Best', 'interpreter', 'latex');
leg.Color = 'none';
leg.NumColumns = 2;
leg.Position(2) = 0.55;
grid on;
box on;
savefig(gcf,'../../matfigs/StocHy_multiple_var_volume.fig','compact')
saveas(gcf,'../../matfigs/StocHy_multiple_var_volume.png','png')

%% Computation time
n_x_vals = 4;
figure(2);
clf;
set(gca,'FontSize',fontSize);
hold on;
yyaxis left;
semilogy(1:n_x_vals, stochy_time(:, 1), marker_Delta_1, ...
    'MarkerSize', markerSize, 'LineWidth', 2, 'LineStyle', ':', ...
    'MarkerFaceColor', marker_Delta_1(1), ...
    'DisplayName', '\texttt{StocHy} (\texttt{IMDP}, $\Delta$=1)');
semilogy(1:n_x_vals, stochy_time(:, 2), marker_Delta_0x2, ...
    'MarkerSize', markerSize, 'LineWidth', 2, 'LineStyle', ':', ...
    'MarkerFaceColor', marker_Delta_0x2(1), ...
    'DisplayName', '\texttt{StocHy} (\texttt{IMDP}, $\Delta$=0.2)');
semilogy(1:n_x_vals, stochy_time(:, 3), marker_Delta_0x15, ...
    'MarkerSize', markerSize, 'LineWidth', 2, 'LineStyle', ':', ...
    'MarkerFaceColor', marker_Delta_0x15(1), ...
    'DisplayName', '\texttt{StocHy} (\texttt{IMDP}, $\Delta$=0.15)');
semilogy(1:n_x_vals, sreachtools_32_time, marker_algo1_32, ...
    'MarkerSize', markerSize, 'LineWidth', 2, 'LineStyle', ':', ...
    'MarkerFaceColor', marker_algo1_32(1), ...
    'DisplayName', 'Alg. 1 (chance const., $|\mathcal{D}|=32$)');
semilogy(1:n_x_vals, sreachtools_8_time, marker_algo1_8, ...
    'MarkerSize', markerSize, 'LineWidth', 2, 'LineStyle', ':', ...
    'MarkerFaceColor', marker_algo1_8(1), 'MarkerEdgeColor', 'k', ...
    'DisplayName', 'Alg. 1 (chance const., $|\mathcal{D}|=8$)');
leg = legend('Location', 'Best', 'interpreter', 'latex', 'AutoUpdate', 'off');
leg.Color = 'none';
leg.Position(2) = 0.46;
semilogy(1:n_x_vals, 0.4*ones(n_x_vals,1), 'k--','LineWidth',2);
semilogy(1:n_x_vals, 3*ones(n_x_vals,1), 'k--','LineWidth', 0.5);
semilogy(1:n_x_vals, 10*ones(n_x_vals,1), 'k--','LineWidth', 0.5);
semilogy(1:n_x_vals, 300*ones(n_x_vals,1), 'k--','LineWidth', 0.5);
semilogy(1:n_x_vals, 1500*ones(n_x_vals,1), 'k--','LineWidth', 0.5);
xlabel('Variance ($\sigma^2$)','interpreter','latex');
ylabel('Computation time (s)','interpreter','latex');
yyaxis right;
ax = gca;
ax.YAxis(1).Color=[0,0,0];
ax.YAxis(2).Color=[0,0,0];
ax.YAxis(1).Scale='log';
ax.YAxis(2).Scale='log';
ax.YAxis(1).Limits = [0.4, 1500];
ax.YAxis(2).Limits = [0.4, 1500];
ax.YAxis(1).TickValues = [0.4, 3, 10, 300, 1500];
ax.YAxis(2).TickValues = [300, 1500];
ax.YAxis(2).TickLabels = {'5 min.' , '25 min.'};
set(gca, 'XTick', 1:n_x_vals);
set(gca, 'XTickLabels', covariance_list);
grid on;
box on;
savefig(gcf,'../../matfigs/StocHy_multiple_var_time.fig','compact')
saveas(gcf,'../../matfigs/StocHy_multiple_var_time.png','png')

figure(1);