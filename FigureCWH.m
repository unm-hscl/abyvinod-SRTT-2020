clear;clc;close all

% 'top'     --- initial velocity of zero
% 'bottom'  --- initial velocity of 0.01
problem_setup = 'top';  
savefigures = 1;
fontSize=35;
markerSize = 20;

if strcmpi(problem_setup, 'top')
    datestr_to_load = '201011_133225';
    loadstr_to_load = './matfiles/CWH_example_%s_zero_vel.mat';
    load(sprintf(loadstr_to_load, datestr_to_load),...
        'safe_set', 'target_set', 'mcarlo_prob_ccc', 'mcarlo_prob_ft', ...
        'extra_info_ccc', 'extra_info_ft','elapsed_time_polytope_ccc', ...
        'elapsed_time_polytope_genzps', 'slice_at_vx_vy', ...
        'underapproximate_stochastic_reach_avoid_polytope_2D_ft', ...
        'underapproximate_stochastic_reach_avoid_polytope_2D_ccc');
    disp('Zero velocity')
    load('./CDC2013_repeatability/Lesser_CCC_Figure8_top.mat', ...
        'x01', 'x02', 'Prob', 'timeSpent'); 
elseif strcmpi(problem_setup, 'bottom')
    datestr_to_load = '201011_130417';
    loadstr_to_load = './matfiles/CWH_example_%s_nonzero_vel.mat';
    load(sprintf(loadstr_to_load, datestr_to_load),...
        'safe_set', 'target_set', 'mcarlo_prob_ccc', 'mcarlo_prob_ft', ...
        'extra_info_ccc', 'extra_info_ft','elapsed_time_polytope_ccc', ...
        'elapsed_time_polytope_genzps', 'slice_at_vx_vy', ...
        'underapproximate_stochastic_reach_avoid_polytope_2D_ft', ...
        'underapproximate_stochastic_reach_avoid_polytope_2D_ccc');
    disp('Non-zero velocity');
    load('./CDC2013_repeatability/Lesser_CCC_Figure8_bottom.mat', ...
        'x01', 'x02', 'Prob', 'timeSpent'); 
else
    throw(SrtInvalidArgsError('problem_setup must be top or bottom'));
end

error_ccc = (mcarlo_prob_ccc - extra_info_ccc(2).opt_reach_prob_i');
error_ft = mcarlo_prob_ft - extra_info_ft.opt_reach_prob_i';
%sreachpoint_prob_ccc);
% error_ft = mcarlo_prob_ft - round(sreachpoint_prob_ft/genzps_options.desired_accuracy) * genzps_options.desired_accuracy;
% error_ft = round((mcarlo_prob_ft - extra_info_ft.opt_reach_prob_i')/genzps_options.desired_accuracy) * genzps_options.desired_accuracy;
fprintf('CCC --- Time: %4.2f | Mean: %1.4f | Std: %1.4f\n', ...
    elapsed_time_polytope_ccc, mean(error_ccc), std(error_ccc));
fprintf('FT  --- Time: %4.2f | Mean: %1.4f | Std: %1.4f\n', ...
    elapsed_time_polytope_genzps, mean(error_ft), std(error_ft));
% fprintf('CCC --- Time: %4.2f\n', elapsed_time_polytope_ccc);
% fprintf('FT --- Time: %4.2f\n', elapsed_time_polytope_genzps);

%% Plotting the cluster as a whole
fig = figure(100);
clf
hold on;
plot(safe_set.slice([3,4], slice_at_vx_vy), 'color', 'y','alpha',0.4);
if strcmpi(problem_setup, 'top')
    plot(target_set.slice([3,4], slice_at_vx_vy), 'color', 'k');
end

%% CDC 2013 (Lesser, Oishi, Erwin): Load the chance-constrained-based solution
%     [c,f]=contour(x01,x02,Prob,[.8 .8],'color','magenta','DisplayName', ...
%         'Grid-based solution [28]','LineWidth',3);
c=contourc(x01,x02,Prob,[.8 .8]);
% Ignore the spurious points that lie outside the safe set via .contains()
% Make the contour points 4D
safe_boundary = safe_set.contains([c; zeros(2, size(c,2))]);
% Add origin to the contour points
contour_boundary_ignore_spurious = [zeros(2,1), c(:, safe_boundary)];
CDC2013_polytope_1 = polyshape(contour_boundary_ignore_spurious');
fprintf('CDC 2013  --- Time: %4.2f\n', timeSpent);
% Partition the contour boundary points into contiguous collection of
% grid points

%% SReachTools solution
if strcmpi(problem_setup, 'top')
    plot(underapproximate_stochastic_reach_avoid_polytope_2D_ft,...
         'color','g','alpha',0.65);
    plot(underapproximate_stochastic_reach_avoid_polytope_2D_ccc,...
         'color','r','alpha',0.8);
else
    plot(underapproximate_stochastic_reach_avoid_polytope_2D_ft,...
     'color','g','alpha',0.65);
    plot(CDC2013_polytope_1,'FaceAlpha',1,'FaceColor','m');
    plot(underapproximate_stochastic_reach_avoid_polytope_2D_ccc,...
         'color','r','alpha',0.8);
end
h_ft=plot(extra_info_ft.xmax(1), extra_info_ft.xmax(2),'gd', 'MarkerFaceColor', 'g', 'MarkerSize', markerSize, 'MarkerEdgeColor', 'k');
h_ccc=plot(extra_info_ccc(2).xmax(1), extra_info_ccc(2).xmax(2), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', markerSize, 'MarkerEdgeColor', 'k');
legend_cell = {'Safe set',...
        'Target set',...
        'Alg. 1 (Fourier transform)',...
        'Alg. 1 (Chance constraint)',...
        '$\bar{x}_\mathrm{anchor}$ (Fourier transform)'...
        '$\bar{x}_\mathrm{anchor}$ (Chance constraint)',...
        'Grid-based solution [27]', ...
        };
if strcmpi(problem_setup, 'top')
    plot(CDC2013_polytope_1,'FaceAlpha',1,'FaceColor','m');
    leg=legend(legend_cell);
else
    plots=get(gca, 'Children');
    leg =legend(plots(flip([4,1,2,3,5,6])), legend_cell{[1,3,4,5,6,7]}); %, {'Second line', 'First line'}
end
set(leg,'Location','EastOutside','interpreter','latex','FontSize',fontSize);
xlabel('x')
ylabel('y')
axis equal;
box on;
grid on;
axis equal
set(gca,'FontSize',fontSize*0.9)
% uistack(h_ft,'top');
% uistack(h_ccc,'top');

if strcmpi(problem_setup, 'top')
    axis([-2.5 2.5 -2 0]);
    if savefigures
        savefig(gcf,'matfigs/CWH_example_ZeroInitVel.fig','compact')
        saveas(gcf,'matfigs/CWH_example_ZeroInitVel.png')
    end
else
    axis([min(x01) max(x01) min(x02) max(x02)])
    if savefigures
        savefig(gcf,'matfigs/CWH_example_NonZeroInitVel.fig','compact')
        saveas(gcf,'matfigs/CWH_example_NonZeroInitVel.png')
    end
end

% %% Stem plot for the vertex error
% figure(3);
% clf
% hold on;
% stem(error_ft, 'bo', 'filled', 'MarkerSize', 15,'LineWidth',3, ...
%     'DisplayName', 'Fourier')
% stem(error_ccc, 'ro', 'filled', 'MarkerSize', 15,'LineWidth',3, ...
%     'DisplayName', 'Chance')
% xlabel('Vertex')
% ylabel('Error');
% % ylim([-0.04 0.04]);
% xlim([0 n_dir_vecs_ccc+1]);
% xticks(1:2:n_dir_vecs_ccc)
% grid on;
% box on;
% set(gca,'FontSize', fontSize);
% legend('Location','NorthWest')
% set(gca,'Position',[0.1300 0.18 0.7750 0.4850])

% end_point_1 = 181;
% end_point_2 = end_point_1+15;
% end_point_3 = end_point_2+4;
% CDC2013_polytope_1 = polyshape(contour_boundary_ignore_spurious(:, ...
%     1:end_point_1)');
% CDC2013_polytope_2 = polyshape(contour_boundary_ignore_spurious(:, ...
%     end_point_1+1:end_point_2)');
% CDC2013_polytope_3 = polyshape(contour_boundary_ignore_spurious(:, ...
%     end_point_2+1:end_point_3)');
% CDC2013_polytope_4 = polyshape(contour_boundary_ignore_spurious(:, ...
%     end_point_3+1:end)');
% plot(CDC2013_polytope_1,'FaceAlpha',0.5,'FaceColor','m','DisplayName', ...
%     'Grid-based solution [28]');
% plot(CDC2013_polytope_2,'FaceAlpha',0.5,'FaceColor','m', ...
%     'HandleVisibility','off');
% plot(CDC2013_polytope_3,'FaceAlpha',0.5,'FaceColor','m', ...
%     'HandleVisibility','off');
% plot(CDC2013_polytope_4,'FaceAlpha',0.5,'FaceColor','m', ...
%     'HandleVisibility','off');
