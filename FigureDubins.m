close all
clc
clear

dropboxpath='D:/Dropbox';
fontSize=20;
% dropboxpath='/datafiles/Dropbox';
% fontSize=30;
date_str_mat = '20180720_152823';
load(strcat(dropboxpath,'/MatFiles/2018TAC_Verification/DubinsCar_example_',date_str_mat,'.mat'));

direction_index_to_plot = 7;
legend_loc = 'EastOutside';


%% Plot the set
figure(101);
clf;
hold on;
for itt=0:time_horizon
    if itt==0
        % Remember the first the tube
        h_target_tube=plot(target_tube_cell{1},'alpha',0.5,'color','y');
    else
        plot(target_tube_cell{itt+1},'alpha',0.08,'LineStyle',':','color','y');
    end            
end
axis equal        
h_poly = plot(underapproximate_stochastic_reach_avoid_polytope_ccc,'color','m');
h_xmax = scatter(xmax_ccc(1), xmax_ccc(2), 100,'ws','filled','MarkerEdgeColor','k');
xlabel('x');
ylabel('y');
axis equal
% axis (1.2*[-box_halflength box_halflength -box_halflength box_halflength]);
axis(axis_vec);
box on;
set(gca,'FontSize',fontSize);
legend_cell = {'Target set at $t=0$','Chance const.', '$\bar{x}_\mathrm{max}$ (Chance const.)'};
legend([h_target_tube, h_poly, h_xmax], legend_cell, 'Location', legend_loc, 'interpreter','latex');
savefig(gcf,'MATLAB_figs/DubinsCar_example_set.fig','compact');
saveas(gcf,'MATLAB_figs/DubinsCar_example_set.png');

%% Plot the trajectories
polyt = underapproximate_stochastic_reach_avoid_polytope_ccc;
if ~isEmptySet(polyt)
    %for direction_index_to_plot = 1:2:no_of_direction_vectors_ccc
        init_state = vertex_poly_ccc(:,direction_index_to_plot);
        input_vec = optimal_input_vector_at_boundary_points_ccc(:,direction_index_to_plot);
        opt_reach_avoid = optimal_reachAvoid_i_ccc(direction_index_to_plot);

        %%
        figure(200+direction_index_to_plot)
        clf
        hold on;
        for itt=0:time_horizon
            if itt==0
                % Remember the first the tube
                h_target_tube=plot(target_tube_cell{1},'alpha',0.5,'color','y');
            else
                plot(target_tube_cell{itt+1},'alpha',0.08,'LineStyle',':','color','y');
            end            
        end
        axis equal        
        h_underapprox=plot(polyt, 'color','m','alpha',0.8);
        h_nominal_traj=scatter(center_box(1,:), center_box(2,:), 50,'ks','filled');
        h_init_state=scatter(init_state(1), init_state(2), 200,'cs','filled');
        legend_cell = {'Target tube','Chance const.','Nominal trajectory','Initial state'};
        %%
        concat_state_realization = generateMonteCarloSims(...
            n_mcarlo_sims,...
            sys,...
            init_state,...
            time_horizon,...
            input_vec);
        % Check if the location is within the target_set or not
        mcarlo_result = target_tube.contains([repmat(init_state,1,n_mcarlo_sims);
                                              concat_state_realization]);
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
                markerString = 'w^-';
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
                    h_good_traj = plot([init_state(1),reshaped_X_vector(1,:)],...
                     [init_state(2),reshaped_X_vector(2,:)],...
                     markerString, 'MarkerSize',10);    
                    legend_cell{end+1} = 'Good trajectory';
                end
            elseif strcmp(markerString,'wd-')
                if red_legend_updated
                    h = plot([init_state(1),reshaped_X_vector(1,:)],...
                     [init_state(2),reshaped_X_vector(2,:)],...
                     markerString, 'MarkerSize',10,'MarkerEdgeColor','k', 'MarkerFaceColor','w');
                    for itt=2:time_horizon+1
                        current_state = concat_state_realization(2*itt-3:2*itt-2,realization_index);
                        if ~target_tube(itt).contains(current_state)
                            h_violation = plot(current_state(1),current_state(2),...
                                      'o', 'MarkerSize',15,'MarkerEdgeColor','k', 'MarkerFaceColor','w');                    
                        end
                    end
                else
                    h_bad_traj = plot([init_state(1),reshaped_X_vector(1,:)],...
                     [init_state(2),reshaped_X_vector(2,:)],...
                     markerString, 'MarkerSize',10,'MarkerEdgeColor','k', 'MarkerFaceColor','w');
                    for itt=2:time_horizon+1
                        current_state = concat_state_realization(2*itt-3:2*itt-2,realization_index);
                        if ~target_tube(itt).contains(current_state)
                            h_violation = plot(current_state(1),current_state(2),...
                                      'o', 'MarkerSize',15,'MarkerEdgeColor','k', 'MarkerFaceColor','w');                    
                        end
                    end
                    red_legend_updated = 1;
                    legend_cell{end+1} = 'Bad trajectory';
                end
            end
        end
        %%
        % Compute and plot the mean trajectory under the optimal open-loop
        % controller from the the vertex under study
        [H_matrix, mean_X_sans_input, ~] =...
         getHmatMeanCovForXSansInput(sys,...
                                     init_state,...
                                     time_horizon);
        optimal_mean_X = mean_X_sans_input + H_matrix * input_vec;
        optimal_mean_trajectory=reshape(optimal_mean_X,sys.state_dim,[]);
        % Plot the optimal mean trajectory from the vertex under study
        h_opt_mean = scatter(...
              [init_state(1), optimal_mean_trajectory(1,:)],...
              [init_state(2), optimal_mean_trajectory(2,:)],...
              30, 'bo', 'filled');
        legend_cell{end+1} = 'Mean trajectory';
        if red_legend_updated
            leg = legend([h_target_tube,h_underapprox,h_nominal_traj,h_init_state,h_good_traj,h_bad_traj,h_opt_mean], legend_cell{:},'Location',legend_loc);%,'interpreter','latex');
        else
            leg = legend([h_target_tube,h_underapprox,h_nominal_traj,h_init_state,h_good_traj,h_opt_mean], legend_cell{:},'Location',legend_loc);%,'interpreter','latex');
        end
        box on;
        grid on;
        axis equal
        axis(axis_vec)
        xlabel('x')
        ylabel('y')
        set(gca,'FontSize',fontSize)
        fprintf(['Open-loop-based lower bound and Monte-Carlo simulation ',...
                         '(%1.0e particles): %1.3f, %1.3f\n'],...
                        n_mcarlo_sims,...
                        opt_reach_avoid,...
                        sum(mcarlo_result)/n_mcarlo_sims);
%     end
end

% savefig(gcf,'MATLAB_figs/DubinsCar_example.fig','compact');
% saveas(gcf,'MATLAB_figs/DubinsCar_example.png');
