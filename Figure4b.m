clear
% dropboxpath='D:/Dropbox';
dropboxpath='/datafiles/Dropbox';
% Run CWH_example with Figure3=1
load(strcat(dropboxpath,'/MatFiles/2018TAC_Verification/Figure4_8.mat'),...
    'safe_set','target_set','xmax_ccc','xmax_ft','slice_at_vx_vy',...
    'vertex_poly_ccc','vertex_poly_ft','optimal_input_vector_at_boundary_points_ccc',...
    'optimal_input_vector_at_boundary_points_ft','optimal_reachAvoid_i_ccc',...
    'optimal_reachAvoid_i_ft','sys','n_mcarlo_sims','time_horizon',...
    'target_tube', 'n_sims_to_plot',...
    'underapproximate_stochastic_reach_avoid_polytope_2D_ccc',...
    'underapproximate_stochastic_reach_avoid_polytope_2D_ft');

direction_index_to_plot = 41;
init_state = vertex_poly_ccc(:,direction_index_to_plot);
%% Plotting a specific one again
figure(200+direction_index_to_plot)
clf
hold on;
h1=plot(safe_set.slice([3,4], slice_at_vx_vy), 'color', 'y');
h2=plot(target_set.slice([3,4], slice_at_vx_vy), 'color', 'k');
h14=plot(underapproximate_stochastic_reach_avoid_polytope_2D_ft, 'color','b','alpha',1);
h4=plot(underapproximate_stochastic_reach_avoid_polytope_2D_ccc, 'color','m','alpha',0.5);
h3=scatter(init_state(1), init_state(2), 200,'cs','filled');
legend_cell = {'Initial state'};
concat_state_realization = generateMonteCarloSims(...
    n_mcarlo_sims,...
    sys,...
    init_state,...
    time_horizon,...
    optimal_input_vector_at_boundary_points_ccc(:,direction_index_to_plot));
% Check if the location is within the target_set or not
mcarlo_result = target_tube.contains(concat_state_realization);
% Plot n_sims_to_plot number of trajectories
green_legend_updated = 0;
red_legend_updated = 0;
traj_indices = floor(n_mcarlo_sims*rand(1,n_sims_to_plot));
for realization_index = traj_indices
    % Check if the trajectory satisfies the reach-avoid objective
    if mcarlo_result(realization_index)
        % Assign green triangle as the marker
        markerString = 'g^-';
    else
        % Assign red asterisk as the marker
        markerString = 'wd';
    end
    % Create [x(t_1) x(t_2)... x(t_N)]
    reshaped_X_vector = reshape(...
        concat_state_realization(:,realization_index),sys.state_dim,[]);
    % This realization is to be plotted
    if strcmp(markerString,'g^-')
        if green_legend_updated
            h = plot([init_state(1),reshaped_X_vector(1,:)],...
             [init_state(2),reshaped_X_vector(2,:)],...
             markerString, 'MarkerSize',10);
        else
            green_legend_updated = 1;
            h5 = plot([init_state(1),reshaped_X_vector(1,:)],...
             [init_state(2),reshaped_X_vector(2,:)],...
             markerString, 'MarkerSize',10);    
            legend_cell{end+1} = 'Good trajectory';
        end
    elseif strcmp(markerString,'wd')
        if red_legend_updated
            h = plot([init_state(1),reshaped_X_vector(1,:)],...
             [init_state(2),reshaped_X_vector(2,:)],'k');
            h = plot([init_state(1),reshaped_X_vector(1,:)],...
             [init_state(2),reshaped_X_vector(2,:)],...
             markerString, 'MarkerSize',10,'MarkerEdgeColor','k', 'MarkerFaceColor','w');            
        else
            h = plot([init_state(1),reshaped_X_vector(1,:)],...
             [init_state(2),reshaped_X_vector(2,:)],'k');
            h6 = plot([init_state(1),reshaped_X_vector(1,:)],...
             [init_state(2),reshaped_X_vector(2,:)],...
             markerString, 'MarkerSize',10,'MarkerEdgeColor','k', 'MarkerFaceColor','w');
            red_legend_updated = 1;
            legend_cell{end+1} = 'Bad trajectory';
        end
    end
end    
% Compute and plot the mean trajectory under the optimal open-loop
% controller from the the vertex under study
[H_matrix, mean_X_sans_input, ~] =...
 getHmatMeanCovForXSansInput(sys,...
                             init_state,...
                             time_horizon);
optimal_mean_X = mean_X_sans_input + H_matrix * optimal_input_vector_at_boundary_points_ccc(:,direction_index_to_plot);
optimal_mean_trajectory=reshape(optimal_mean_X,sys.state_dim,[]);
% Plot the optimal mean trajectory from the vertex under study
h7 = scatter(...
      [init_state(1), optimal_mean_trajectory(1,:)],...
      [init_state(2), optimal_mean_trajectory(2,:)],...
      30, 'bo', 'filled');
legend_cell{end+1} = 'Mean trajectory';
if red_legend_updated
    leg = legend([h3,h5,h6,h7], legend_cell{:},'Location','NorthWest','interpreter','latex');
else
    leg = legend([h3,h5,h7], legend_cell{:},'Location','NorthWest','interpreter','latex');
end
box on;
grid on;
axis equal
xlabel('x')
ylabel('y')
set(gca,'FontSize',20)
axis([-1.2 0.25 -1.2 0.1])
fprintf(['Open-loop-based lower bound (CC) and Monte-Carlo simulation ',...
                 '(%1.0e particles): %1.3f, %1.3f\n'],...
                n_mcarlo_sims,...
                optimal_reachAvoid_i_ccc(direction_index_to_plot),...
                sum(mcarlo_result)/n_mcarlo_sims);
savefig(gcf,'MATLAB_figs/CWH_example_Figure4b.fig','compact');
saveas(gcf,'MATLAB_figs/CWH_example_Figure4b.png');