clear
close all
clc

time_horizon = 50;
time_const = 3/4*time_horizon;
init_heading = pi/4;
sampling_time = 0.1;
box_halflength = 2;
omega = pi/time_horizon/sampling_time;
turning_rate = omega*ones(time_horizon,1);
dist_cov = 0.001;
probability_threshold_of_interest = 0.8;
no_of_direction_vectors_ccc = 10;
direction_index_to_plot = 6;
n_mcarlo_sims = 1e5;
n_sims_to_plot = 10;
axis_vec = [-11 6 -box_halflength*1.5 10];
legend_loc = 'West';

%% LTV system definition
[sys, heading_vec] = getDubinsCarLtv('add-dist',...
turning_rate,...
init_heading,...
sampling_time,...
Polyhedron('lb',0,'ub',5),...
eye(2),...
RandomVector('Gaussian',zeros(2,1), dist_cov * eye(2)));

target_tube_cell = cell(time_horizon + 1,1);

%% Target tube definition
figure(100);clf;hold on
for itt=0:time_horizon
    center_box = -5 * [cos(heading_vec(itt+1)+pi/2)-cos(init_heading+pi/2);
                       sin(heading_vec(itt+1)+pi/2)-sin(init_heading+pi/2)];
    target_tube_cell{itt+1} = Polyhedron('lb',center_box - box_halflength * exp(- itt/time_const), 'ub', center_box + box_halflength*exp(- itt/time_const));
    plot(target_tube_cell{itt+1},'alpha',0.5);
end
axis equal
axis(axis_vec)
target_tube = TargetTube(target_tube_cell{:});

%% Set of direction vectors
theta_vector_ccc = linspace(0, 2*pi, no_of_direction_vectors_ccc+1);
theta_vector_ccc = theta_vector_ccc(1:end-1);
set_of_direction_vectors_ccc = [cos(theta_vector_ccc); 
                                sin(theta_vector_ccc)];
timer_polytope_ccc = tic;
[underapproximate_stochastic_reach_avoid_polytope_ccc,...
 optimal_input_vector_at_boundary_points_ccc,...
 xmax_ccc,...
 optimal_input_vector_for_xmax_ccc,...
 maximum_underapproximate_reach_avoid_probability_ccc,...
 optimal_theta_i_ccc,...
 optimal_reachAvoid_i_ccc,...
 vertex_poly_ccc,...
 R,...
 max_error] = getUnderapproxStochReachAvoidSet(...
    sys, ...
    target_tube, ...
    [], ...
    probability_threshold_of_interest, ...
    set_of_direction_vectors_ccc,...
    'ccc');
elapsed_time_polytope_ccc = toc(timer_polytope_ccc);
fprintf('Time taken for computing the polytope (CCC): %1.3f s\n', elapsed_time_polytope_ccc);

%% Save the data
save('DubinsCar_example.mat');

%% Plot the set
figure(101);
clf;
hold on;
plot(target_tube(1));
plot(underapproximate_stochastic_reach_avoid_polytope_ccc,'color','b');
axis equal
axis (1.2*[-box_halflength box_halflength -box_halflength box_halflength]);
box on;
legend('Target set at t=0','Stochastic reach set','Location','SouthEast');

%% Plot the trajectories
polyt = underapproximate_stochastic_reach_avoid_polytope_ccc;
if ~isEmptySet(polyt)
    %for direction_index_to_plot = 1:2:no_of_direction_vectors_ccc
        init_state = vertex_poly_ccc(:,direction_index_to_plot);
        input_vec = optimal_input_vector_at_boundary_points_ccc(:,direction_index_to_plot);
        opt_reach_avoid = optimal_reachAvoid_i_ccc(direction_index_to_plot);

        figure(200+direction_index_to_plot)
        clf
        hold on;
        for itt=0:time_horizon
            if itt==0
                % Remember the first the tube
                h1 = plot(target_tube_cell{itt+1},'alpha',0.5);
            else
                plot(target_tube_cell{itt+1},'alpha',0.5);
            end
        end
        axis equal
        h3=scatter(init_state(1), init_state(2), 200,'cs','filled');
        h4=plot(polyt, 'color','m','alpha',0.5);
        legend_cell = {'Initial state'};
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
                    h5 = plot([init_state(1),reshaped_X_vector(1,:)],...
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
                            h8 = plot(current_state(1),current_state(2),...
                                      'o', 'MarkerSize',15,'MarkerEdgeColor','k', 'MarkerFaceColor','w');                    
                        end
                    end
                else
                    h6 = plot([init_state(1),reshaped_X_vector(1,:)],...
                     [init_state(2),reshaped_X_vector(2,:)],...
                     markerString, 'MarkerSize',10,'MarkerEdgeColor','k', 'MarkerFaceColor','w');
                    for itt=2:time_horizon+1
                        current_state = concat_state_realization(2*itt-3:2*itt-2,realization_index);
                        if ~target_tube(itt).contains(current_state)
                            h8 = plot(current_state(1),current_state(2),...
                                      'o', 'MarkerSize',15,'MarkerEdgeColor','k', 'MarkerFaceColor','w');                    
                        end
                    end
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
        optimal_mean_X = mean_X_sans_input + H_matrix * input_vec;
        optimal_mean_trajectory=reshape(optimal_mean_X,sys.state_dim,[]);
        % Plot the optimal mean trajectory from the vertex under study
        h7 = scatter(...
              [init_state(1), optimal_mean_trajectory(1,:)],...
              [init_state(2), optimal_mean_trajectory(2,:)],...
              30, 'bo', 'filled');
        legend_cell{end+1} = 'Mean trajectory';
        if red_legend_updated
            leg = legend([h3,h5,h6,h7], legend_cell{:},'Location',legend_loc);%,'interpreter','latex');
        else
            leg = legend([h3,h5,h7], legend_cell{:},'Location',legend_loc);%,'interpreter','latex');
        end
        box on;
        grid on;
        axis equal
        axis(axis_vec)
        xlabel('x')
        ylabel('y')
        set(gca,'FontSize',20)
        fprintf(['Open-loop-based lower bound and Monte-Carlo simulation ',...
                         '(%1.0e particles): %1.3f, %1.3f\n'],...
                        n_mcarlo_sims,...
                        opt_reach_avoid,...
                        sum(mcarlo_result)/n_mcarlo_sims);
%     end
end