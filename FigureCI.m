clear
clc
close all

datestr_to_load = '201011_115942';
load(sprintf('./matfiles/CI_example_%s.mat', datestr_to_load));
fontSize=30;
titleFontSize = 25;
savefigures = 1;

%% Open-loop underapproximation
figure(3);
clf
hold on
plot(target_tube(1).slice(n_zeroed+1:n_intg,zeros(n_intg-n_zeroed,1)),'color','y','alpha',0.4);
plot(target_tube(time_horizon + 1).slice(n_zeroed+1:n_intg,zeros(n_intg-n_zeroed,1)),'color','k','alpha',1);
color_string = ['c','m','b'];
for i=1:2:length(alpha_vec)
    plot(underapproximate_stochastic_reach_avoid_polytope_ccc(i).slice(n_zeroed+1:n_intg,zeros(n_intg-n_zeroed,1)),'color',color_string(i),'alpha',0.8);
end
axis equal
box on
xlabel('$x_1$','interpreter','latex')
ylabel('$x_2$','interpreter','latex')
set(gca,'FontSize',fontSize);
axis(1.1*xmax_safe*[-1 1 -1 1]);
legend_cell = {'Safe set','Target set','Algorithm 1 ($\alpha=0.6$)','Algorithm 1 ($\alpha=0.9$)'};
leg=legend(legend_cell{:},'interpreter','latex');
set(leg,'Location','SouthOutside');
title('c)','FontSize',titleFontSize)
set(gca, 'Position', [0.1300    0.4969    0.7750    0.4515]);
if savefigures
    savefig(gcf,'./matfigs/CI_multiple.fig','compact')
    saveas(gcf,'./matfigs/CI_multiple.png','png')
end
% leg=legend('Safe set','Target set','Open loop underapprox. (0.6)','Open loop underapprox. (0.9)');
% set(leg,'Location','EastOutside');
% colorbar
% colormap winter;
% caxis([0 1]);

%% Computation of 2D slices as well as the volume difference
underapproximate_stochastic_reach_avoid_polytope_ccc_2D_plot = underapproximate_stochastic_reach_avoid_polytope_ccc(2).slice(n_zeroed+1:n_intg,zeros(n_intg-n_zeroed,1));
interp_set_2D_plot = interp_set.slice(n_zeroed+1:n_intg,zeros(n_intg-n_zeroed,1));
vol_poly = volume(underapproximate_stochastic_reach_avoid_polytope_ccc_2D_plot);
vol_interp_poly = volume(interp_set_2D_plot);

%% Interpolation comparison
figure(4)
clf
hold on
plot(target_tube(1).slice(n_zeroed+1:n_intg,zeros(n_intg-n_zeroed,1)),'color','y','alpha',0.4);
plot(target_tube(time_horizon + 1).slice(n_zeroed+1:n_intg,zeros(n_intg-n_zeroed,1)),'color','k');
plot(underapproximate_stochastic_reach_avoid_polytope_ccc_2D_plot,'color',color_string(2));
plot(interp_set_2D_plot,'color','w');
axis equal
box on
xlabel('$x_1$','interpreter','latex')
ylabel('$x_2$','interpreter','latex')
set(gca,'FontSize',fontSize);
axis(1.1*xmax_safe*[-1 1 -1 1]);
legend_cell = {'Safe set','Target set','Algorithm 1 ($\alpha=0.85$)','Algorithm 2 ($\beta=0.85$)'};
leg=legend(legend_cell{:},'interpreter','latex');
set(leg,'Location','SouthOutside');
title('d)','FontSize',titleFontSize);
set(gca, 'Position', [0.1300    0.4969    0.7750    0.4515]);
if savefigures
    savefig(gcf,'./matfigs/CI_interp.fig','compact')
    saveas(gcf,'./matfigs/CI_interp.png','png')
end

fprintf('Computation time\nAlgorithm 1: %s\nInterp: %1.3f\n\n', num2str(elapsed_time_polytope_ccc','$%1.2f$ & '), elapsed_time_interp);
disp('Ratio between volumes of the 2D slices (interp/original)');
disp(vol_interp_poly/vol_poly);
disp('Difference between volumes of the 2D slices');
disp(vol_poly - vol_interp_poly)
disp('Ratio of diff with volume of safe set');
disp((vol_poly - vol_interp_poly)/(2*xmax_safe^2))
fprintf('\n>>> Notes\n')
disp('Remember to minimize the ribbon above for getting the size used in the paper.');
disp('Hardcoded position to [0.1300    0.4969    0.7750    0.4515]');
