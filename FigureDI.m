clear;clc;close all

datestr_to_load = '201011_111336';
load(sprintf('./matfiles/DI_example_%s.mat', datestr_to_load));
savefigures = 1;
fontSize=30;
titleFontSize = 25;

grid_probability_mat = reshape(prob_x, length(x),[]);
%% Open-loop underapproximation
figure(1);
clf
hold on
color_string = ['c','g','b'];
color_string_DP = ['r','r','m'];
plot(safe_set,'color','y','alpha',0.4);
plot(poly_array(1),'color',color_string_DP(1))
plot(underapproximate_stochastic_reach_avoid_polytope_ccc(1),'color',color_string(1));
plot(poly_array(3),'color',color_string_DP(3))
plot(underapproximate_stochastic_reach_avoid_polytope_ccc(3),'color',color_string(3));
axis equal
box on
xlabel('$x_1$','interpreter','latex')
ylabel('$x_2$','interpreter','latex')
set(gca,'FontSize',fontSize);
axis(1.1*[-xmax(1) xmax(1) -xmax(2) xmax(2)]);
legend_cell = {'Safe set','Dyn. prog. ($\alpha=0.6$)', ...
    'Algorithm 1 ($\alpha=0.6$)','Dyn. prog. ($\alpha=0.9$)', ...
    'Algorithm 1 ($\alpha=0.9$)'};
leg=legend(legend_cell{:},'interpreter','latex');
set(leg,'Location','SouthOutside');
title('a)','FontSize',titleFontSize);
set(gca, 'Position', [0.1300    0.4969    0.7750    0.4515]);
if savefigures
    savefig(gcf,'./matfigs/DI_DPvsCCC.fig','compact')
    saveas(gcf,'./matfigs/DI_DPvsCCC.png','png')
end
% [c,h]=contour(x, x, grid_probability_mat, alpha_vec([1,3]),'LineWidth',3);
% fprintf('Choose contours and set 0.9 label colors to white\n');
% clabel(c,h,'manual','FontSize',fontSize,'color','k','FontWeight','bold','BackgroundColor','w');
% colorbar
% colormap winter;
% caxis([0 1]);

%% Interpolation comparison
figure(2)
clf
hold on
h_safe = plot(safe_set,'color','y','alpha',0.4);
h_contour = plot(poly_array(2),'color',color_string_DP(2));
h_interp_DP = plot(interp_set_DP,'color','c');
h_OL = plot(underapproximate_stochastic_reach_avoid_polytope_ccc(2),'color','m','alpha',1);
h_interp_OL = plot(interp_set,'color','b','alpha',1);
axis equal
box on
xlabel('$x_1$','interpreter','latex')
ylabel('$x_2$','interpreter','latex')
set(gca,'FontSize',fontSize);
axis(1.1*[-xmax(1) xmax(1) -xmax(2) xmax(2)]);
legend_cell = {'Safe set','Dyn. prog. ($\alpha=0.85$)', ...
    'Interpolation ($\beta=0.85$)','Algorithm 1 ($\alpha=0.85$)', ...
    'Algorithm 2 ($\beta=0.85$)'};
leg=legend(legend_cell{:},'interpreter','latex');
set(leg,'Location','SouthOutside');
title('b)','FontSize',titleFontSize);
set(gca, 'Position', [0.1300    0.4969    0.7750    0.4515]);
if savefigures
    savefig(gcf,'./matfigs/DI_interp.fig','compact')
    saveas(gcf,'./matfigs/DI_interp.png','png')
end
% [c,h_contour]=contour(x, x, grid_probability_mat, alpha_vec([2,2]),'LineWidth',3);
% clabel(c,h_contour,'FontSize',2*fontSize/3);
% fprintf('Choose contours and set 0.9 label colors to white\n');
% clabel(c,h_contour,'manual','FontSize',fontSize,'color','k','FontWeight','bold','BackgroundColor','w');

%% Compute times
clc
fprintf('Computation time\nAlgorithm 1: %s\nInterp: %1.3f\n\n', num2str(elapsed_time_polytope_ccc','$%1.2f$ & '), elapsed_time_interp);
fprintf('Computation time\nDP: %1.2f\nInterp: %1.3f\n\n', elapsed_time_DP_total, elapsed_time_interp_DP);
disp('Ratio between volumes of the 2D slices (interp/original)');
vol_poly = volume(underapproximate_stochastic_reach_avoid_polytope_ccc(2));
vol_interp_poly = volume(interp_set);
disp(vol_interp_poly/vol_poly);
fprintf('\n>>> Notes\n')
disp('Remember to minimize the ribbon above for getting the size used in the paper.');
disp('Used the high memory usage setting in dynamic programming');
disp('Hardcoded position to [0.1300    0.4969    0.7750    0.4515]');
