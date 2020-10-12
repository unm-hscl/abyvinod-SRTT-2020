clear;clc;close all

savefigures = 1;                        % For maximized figure window
fontSize=40;
legFontSize = 35;%31.5;
titleFontSize = 25;

% Extract Algorithm 1 performance
datestr_to_load = '201011_173311';
load(sprintf('./Figure8_data/StocHy_example_%s.mat', datestr_to_load), ...
    'elapsed_time_polytope_ccc', 'extra_info_cell', 'elapsed_time_Wmax_ccc');
W_max = zeros(length(elapsed_time_polytope_ccc),1);
for indx = 1:length(elapsed_time_polytope_ccc)
    W_max(indx) = extra_info_cell{indx}(1).xmax_reach_prob;
end
marker_algo1 = 'bo-';
marker_Wmax = 'bv-';
marker_Delta_1 = 'rs-';
marker_Delta_0x2 = 'md-';
marker_Delta_0x15 = 'cp-';
markerSize = 15;


cd('./Figure8_data/')
% Extract runtime and V_0^I(x) for grid delta = 1
cd('results_scalability_grid_1');
grid_points = [4,14,30,62,126,254,510,1022,2046];
% Get elapsed time
delta1_filenames = dir('IMDP_Runtime_*');
elapsed_time_scalability_grid_1 = zeros(length(delta1_filenames), 1);
for indx = 1:length(delta1_filenames)
    filename = delta1_filenames(indx).name;
    split_contents = split(filename, 'Runtime_');
    split_split_contents = split(split_contents{2}, '_');
    number_of_grid_pts = str2num(split_split_contents{1});
    elapsed_time_scalability_grid_1(number_of_grid_pts == grid_points) = ...
        csvread(filename);
end
% Get the max lower bound
delta1_filenames = dir('IMDP_Solution_*');
V0_lb_max_grid_1 = zeros(length(delta1_filenames), 1);
V0_ub_max_grid_1 = zeros(length(delta1_filenames), 1);
for indx = 1:length(delta1_filenames)
    filename = delta1_filenames(indx).name;
    f = fopen(filename);
    error_bounds = textscan(f,'%f %f');
    fclose(f);
    [V0_lb_max_grid_1(indx), max_index] = max(error_bounds{1});
    V0_ub_max_grid_1(indx) = error_bounds{2}(max_index);    
end
cd('..');

% Extract runtime and V_0^I(x) for grid delta = 0.2
cd('results_scalability_grid_0x2');
grid_points = [100,1110];
% Get the elapsed time
delta0x2_filenames = dir('IMDP_Runtime_*');
elapsed_time_scalability_grid_0x2 = zeros(length(delta0x2_filenames), 1);
for indx = 1:length(delta0x2_filenames)
    filename = delta0x2_filenames(indx).name;
    split_contents = split(filename, 'Runtime_');
    split_split_contents = split(split_contents{2}, '_');
    number_of_grid_pts = str2num(split_split_contents{1});
    elapsed_time_scalability_grid_0x2(number_of_grid_pts == grid_points) = ...
        csvread(filename);
end
% Get the max lower bound
delta0x2_filenames = dir('IMDP_Solution_*');
V0_lb_max_grid_0x2 = zeros(length(delta0x2_filenames), 1);
V0_ub_max_grid_0x2 = zeros(length(delta0x2_filenames), 1);
for indx = 1:length(delta0x2_filenames)
    filename = delta0x2_filenames(indx).name;
    f = fopen(filename);
    error_bounds = textscan(f,'%f %f');
    fclose(f);
    [V0_lb_max_grid_0x2(indx), max_index] = max(error_bounds{1});
    V0_ub_max_grid_0x2(indx) = error_bounds{2}(max_index);
end
bad_dim_0x2 = (V0_lb_max_grid_0x2 > 1);
if sum(bad_dim_0x2) > 0
    fprintf('Found %d bad dimensions for 0x2\n', sum(bad_dim_0x2));
end
V0_lb_max_grid_0x2(bad_dim_0x2) = 0;
cd('..');

% Extract runtime and V_0^I(x) for grid delta = 0.2
cd('results_scalability_grid_0x15');
grid_points = [169, 2548];
% Get the elapsed time
delta0x15_filenames = dir('IMDP_Runtime_*');
elapsed_time_scalability_grid_0x15 = zeros(length(delta0x15_filenames), 1);
for indx = 1:length(delta0x15_filenames)
    filename = delta0x15_filenames(indx).name;
    split_contents = split(filename, 'Runtime_');
    split_split_contents = split(split_contents{2}, '_');
    number_of_grid_pts = str2num(split_split_contents{1});
    elapsed_time_scalability_grid_0x15(number_of_grid_pts == grid_points) = ...
        csvread(filename);
end
% Get the max lower bound
delta0x15_filenames = dir('IMDP_Solution_*');
V0_lb_max_grid_0x15 = zeros(length(delta0x15_filenames), 1);
V0_ub_max_grid_0x15 = zeros(length(delta0x15_filenames), 1);
for indx = 1:length(delta0x15_filenames)
    filename = delta0x15_filenames(indx).name;
    f = fopen(filename);
    error_bounds = textscan(f,'%f %f');
    fclose(f);
    [V0_lb_max_grid_0x15(indx), max_index] = max(error_bounds{1});
    V0_ub_max_grid_0x15(indx) = error_bounds{2}(max_index);
end
bad_dim_0x15 = (V0_lb_max_grid_0x15 > 1);
if sum(bad_dim_0x15) > 0
    fprintf('Found %d bad dimensions for 0x15\n', sum(bad_dim_0x15));
end
V0_lb_max_grid_0x15(bad_dim_0x15) = 0;
cd('..');
cd('..');

%% Computation time
figure(1);
yyaxis left;
semilogy(2:length(elapsed_time_polytope_ccc)+1, ... 
    elapsed_time_polytope_ccc,marker_algo1,'MarkerSize', markerSize,  ...
    'LineWidth', 2, 'LineStyle', ':', 'MarkerFaceColor', marker_algo1(1), ...
    'DisplayName', 'Alg. 1 (chance const., $|\mathcal{D}|=2^n$)');
set(gca,'FontSize',fontSize);
hold on;
% semilogy(2:length(elapsed_time_polytope_ccc)+1, ... 
%     elapsed_time_Wmax_ccc,marker_Wmax,'MarkerSize', markerSize,  ...
%     'LineWidth', 2, 'LineStyle', ':', 'MarkerFaceColor', marker_Wmax(1), ...
%     'DisplayName', 'Solve (21), $W^\ast_0(\overline{x}_\mathrm{max})$'); %'$\sup_{x\in\mathcal{X}} W_0(x)$ via (21)');
semilogy(2:length(elapsed_time_scalability_grid_1)+1, ...
    elapsed_time_scalability_grid_1, marker_Delta_1,'MarkerSize', markerSize,  ...
    'LineWidth', 2, 'LineStyle', ':', 'MarkerFaceColor', marker_Delta_1(1), ...
    'DisplayName', '\texttt{StocHy} (\texttt{IMDP}, $\Delta$=1)');
semilogy(2:length(elapsed_time_scalability_grid_0x2)+1, ...
    elapsed_time_scalability_grid_0x2,marker_Delta_0x2,'MarkerSize', markerSize,  ...
    'LineWidth', 2, 'LineStyle', ':', 'MarkerFaceColor', marker_Delta_0x2(1), ...
    'DisplayName', '\texttt{StocHy} (\texttt{IMDP}, $\Delta$=0.2)');
semilogy(2:length(elapsed_time_scalability_grid_0x15)+1, ...
    elapsed_time_scalability_grid_0x15,marker_Delta_0x15,'MarkerSize', markerSize,  ...
    'LineWidth', 2, 'LineStyle', ':', 'MarkerFaceColor', marker_Delta_0x15(1), 'MarkerEdgeColor', 'k', ...
    'DisplayName', '\texttt{StocHy} (\texttt{IMDP}, $\Delta$=0.15)');
leg=legend('Location','best','interpreter','latex','AutoUpdate','off', ...
    'FontSize', legFontSize);
% leg.NumColumns = 3;
semilogy(2:10, 60*ones(9,1), 'k--','LineWidth',2);
semilogy(2:10, 1200*ones(9,1), 'k--','LineWidth',2);
semilogy(2:10, 3600*ones(9,1), 'k--','LineWidth',2);
set(gca,'XTick',2:10);
xlim([2,10]);
xlabel('Dimension (n)','interpreter','latex');
ylabel('Computation time (s)','interpreter','latex');
yyaxis right;
ax = gca;
ax.YAxis(1).Color=[0,0,0];
ax.YAxis(2).Color=[0,0,0];
ax.YAxis(1).Scale='log';
ax.YAxis(2).Scale='log';
ax.YAxis(1).Limits = [0.01, 3600];
ax.YAxis(2).Limits = [0.01, 3600];
ax.YAxis(1).TickValues = [0.01,0.1, 1,10,60,300,1200,3600];
ax.YAxis(2).TickValues = [60, 1200, 3600];
ax.YAxis(2).TickLabels = {'1 min.', '20 min.' , '1 hr.'};
grid on;
box on;
ax.YAxis(1).Color=[0,0,0];
ax.YAxis(2).Color=[0,0,0];
set(gca,'Position', [0.1300    0.1599    0.7505    0.6651]);
if savefigures
    savefig(gcf,'../matfigs/StocHy_multiple_dim_compute_time.fig','compact')
    saveas(gcf,'../matfigs/StocHy_multiple_dim_compute_time.png','png')
end

%% V max plot
figure(2);
clf;
hold on;
set(gca,'FontSize',fontSize);
semilogy(2:length(elapsed_time_polytope_ccc)+1, W_max, marker_algo1, ...
    'LineStyle', ':', 'LineWidth', 2, 'MarkerSize', markerSize,  ...
    'MarkerFaceColor', marker_algo1(1), 'DisplayName', ...
    '$\sup_{\overline{x}\in\mathcal{X}} W^\ast_0(\overline{x})=W^\ast_0(\overline{x}_\mathrm{max})$ Alg. 1 (chance const.)');
plot(2:length(V0_lb_max_grid_1)+1, V0_lb_max_grid_1, marker_Delta_1, ...
    'LineStyle', ':', 'LineWidth', 2, 'MarkerSize', markerSize,  ...
    'MarkerFaceColor', marker_Delta_1(1), 'DisplayName', ...
    '$\sup_{\overline{x}\in\mathcal{X}_\mathrm{grid}} V_0^I(\overline{x};\Delta)$ \texttt{StocHy} (\texttt{IMDP}, $\Delta$=1)');
plot(2:length(V0_lb_max_grid_0x2)+1, V0_lb_max_grid_0x2, marker_Delta_0x2, ...
    'LineStyle', ':', 'LineWidth', 2, 'MarkerSize', markerSize,  ...
    'MarkerFaceColor', marker_Delta_0x2(1), 'DisplayName', ...
    '$\sup_{\overline{x}\in\mathcal{X}_\mathrm{grid}} V_0^I(\overline{x};\Delta)$ \texttt{StocHy} (\texttt{IMDP}, $\Delta$=0.2)');
plot(2:length(V0_lb_max_grid_0x2)+1, V0_lb_max_grid_0x15, marker_Delta_0x15, ...
    'LineStyle', ':', 'LineWidth', 2, 'MarkerSize', markerSize,  ...
    'MarkerFaceColor', marker_Delta_0x15(1), 'MarkerEdgeColor', 'k', 'DisplayName', ...
    '$\sup_{\overline{x}\in\mathcal{X}_\mathrm{grid}} V_0^I(\overline{x};\Delta)$ \texttt{StocHy} (\texttt{IMDP}, $\Delta$=0.15)');
leg=legend('Position', [0.2790 0.2159 0.5570 0.2656],'interpreter','latex', ...
    'AutoUpdate','off', 'FontSize', legFontSize);
set(gca,'XTick',2:10);
ylim([0,1]);
xlim([2,10]);
xlabel('Dimension (n)','interpreter','latex');
ylabel('Maximum safety probability','interpreter','latex','fontSize',38.5);
grid on;
box on;
set(gca,'Position', [0.1300    0.1599    0.7505    0.6651]);
disp('Hardcoded position to [0.1300    0.1599    0.7505    0.6651]');
if savefigures
    savefig(gcf,'../matfigs/StocHy_multiple_dim_max_V.fig','compact')
    saveas(gcf,'../matfigs/StocHy_multiple_dim_max_V.png','png')
end

% disp('Grid size 1');
% disp([V0_lb_max_grid_1, V0_ub_max_grid_1]);
% disp('Grid size 0.2');
% disp([V0_lb_max_grid_0x2, V0_ub_max_grid_0x2]);
% disp('Grid size 0.15');
% disp([V0_lb_max_grid_0x15, V0_ub_max_grid_0x15]);
