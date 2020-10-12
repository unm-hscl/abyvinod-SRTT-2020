close all
clc
clear

datestr_to_load = '201011_145310';
load(sprintf('./matfiles/Car_example_%s.mat', datestr_to_load));

legend_loc = 'EastOutside';
savefigures = 1;
fontSize = 35;
markerSize = 15;

%% Plot the set
figure(101);
clf;
hold on;
a = gca;
n_child = length(a.Children);
for itt=0:time_horizon
    if itt==0
        % Remember the first the tube
        h_target_tube = plot(target_tube_cell{1},'alpha',0.5,'color','y');
    else
        plot(target_tube_cell{itt+1},'alpha',0.1,'LineStyle',':','color','y');
    end            
end
axis equal        
h_poly = plot(ccc_polytope,'color','m');
h_xmax = plot(extra_info(2).xmax(1), extra_info(2).xmax(2), 'bd', 'MarkerFaceColor', 'b', 'MarkerSize', markerSize, 'MarkerEdgeColor', 'k');

xlabel('x');
ylabel('y');
axis equal
axis ([-6.5 10 -5 22]);
box on;

% Plot the initial state
h_init = plot(X_test(1), X_test(2), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', markerSize, 'MarkerEdgeColor', 'k');
% Plot the convex hull of the spread of the points
[~, hmc]=polytopesFromMonteCarloSims(X_test, 2, [1,2], ...
    {'color','c','alpha',0.6});
a.Children(end).Annotation.LegendInformation.IconDisplayStyle = 'on';

%% Legend
legend_cell = {'Target tube','Alg. 1 (Chance constraint)', ...
    '$\bar{x}_\mathrm{anchor}$ (Chance constraint)','Initial state used for validation','Simulated spread of trajectories'};
legend([h_target_tube, h_poly, h_xmax, h_init, hmc], legend_cell, 'Location',...
    legend_loc, 'interpreter','latex','FontSize',fontSize);
set(gca,'FontSize',fontSize*0.9)
if savefigures
    savefig(gcf,'./matfigs/Car_example_set.fig','compact');
    saveas(gcf,'./matfigs/Car_example_set.png');
end
 
% %% Stem plot of Monte-Carlo plot
% figure(102);
% clf
% % extra_info(2).opt_reach_prob_i
% stem(extra_info(info_indx).opt_reach_prob_i, 'bo', 'filled', ...
%     'MarkerSize', 15,'LineWidth',3, 'DisplayName', 'Lower bound (Algorithm. 1)')
% hold on;
% stem(mcarlo_prob, 'rd', 'filled', 'MarkerSize',15,'LineWidth', 3, ...
%     'DisplayName', 'Simulated reach probability')
% xlabel('Vertex')
% ylabel('Reach probability');
% ylim([0.75 1.035]);
% xlim([0 no_of_direction_vectors_ccc+1]);
% xticks(1:no_of_direction_vectors_ccc)
% grid on;
% set(gca,'FontSize', fontSize);
% legend('Location','NorthWest')
% set(gca,'Position',[0.1300 0.18 0.7750 0.4850])
% if savefigures
%     savefig(gcf,'./matfigs/Car_example_stem.fig','compact')
%     saveas(gcf,'./matfigs/Car_example_stem.png')
% end
%
% % % Redraw the borders
% % n_child = length(a.Children);
% % for itt=1:time_horizon+1
% %     h = plot(target_tube_cell{itt},'alpha',0,'LineStyle',':');
% % end

error_ccc = mcarlo_prob' - extra_info(2).opt_reach_prob_i;
fprintf('CCC --- Time: %4.2f | Mean: %1.4f | Std: %1.4f\n', ...
    elapsed_time_polytope_ccc, mean(error_ccc), std(error_ccc));
