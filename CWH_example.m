%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Name of the programmer: Abraham %
% Date: 2018-07-08                %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Purpose
% Verification of a satellite rendezvous and docking problem

%% Notes
% 1. This script has been modified from
%   SReachTools/examples/FtCVXUnderapproxVerifyCWH

clear
clc
close all

figure4 = 0;  % Setting this to zero will do Kendra's version with initial velocity of zero

%% Dynamics model for the deputy relative to the chief spacecraft
if figure4
    umax=0.1;
    figsave_str = 'zero';
else
    umax=0.01;
    figsave_str = 'nonzero';
end
mean_disturbance = zeros(4,1);
covariance_disturbance = diag([1e-4, 1e-4, 5e-8, 5e-8]);
% Define the CWH (planar) dynamics of the deputy spacecraft relative to the
% chief spacecraft as a LtiSystem object
sys = getCwhLtiSystem(4,...
                      Polyhedron('lb', -umax*ones(2,1),...
                                 'ub',  umax*ones(2,1)),...
                      RandomVector('Gaussian',...
                                   mean_disturbance,...
                                   covariance_disturbance));

%% Problem parameters
time_horizon=5;          % Stay within a line of sight cone for 4 time steps and 
                         % reach the target at t=5
                         % Safe Set --- LoS cone
probability_threshold_of_interest = 0.8;     % Probability threshold of interest
no_of_direction_vectors = 16;  % Increase for a tighter polytopic representation 
                               % at the cost of higher computation time
n_mcarlo_sims = 1e5;
n_sims_to_plot = 5;

%% Target set and safe set creation
% Safe set definition --- LoS cone |x|<=y and y\in[0,ymax] and |vx|<=vxmax and
% |vy|<=vymax
ymax=2;
vxmax=0.5;
vymax=0.5;
A_safe_set = [1, 1, 0, 0;           
             -1, 1, 0, 0; 
              0, -1, 0, 0;
              0, 0, 1,0;
              0, 0,-1,0;
              0, 0, 0,1;
              0, 0, 0,-1];
b_safe_set = [0;
              0;
              ymax;
              vxmax;
              vxmax;
              vymax;
              vymax];
safe_set = Polyhedron(A_safe_set, b_safe_set);

% Target set --- Box [-0.1,0.1]x[-0.1,0]x[-0.01,0.01]x[-0.01,0.01]
target_set = Polyhedron('lb', [-0.1; -0.1; -0.01; -0.01],...
                        'ub', [0.1; 0; 0.01; 0.01]);
% Target tube definition
target_tube = TargetTube('reach-avoid', safe_set, target_set, time_horizon);


if figure4
    % OPTION 1: Initial velocity is zero
    slice_at_vx_vy = zeros(2,1);                 % Initial velocities of interest
else
    % OPTION 2: Initial velocity is non-zero
    slice_at_vx_vy = 0.01*ones(2,1);             % The initial velocities of interest
end
% Construct the set of direction vectors
%theta_vector = [pi/4, pi/2, 3*pi/4,...
                %linspace(pi, 5/4*pi, floor(no_of_direction_vectors-3)/2),...
                %-pi/2 , linspace(-pi/4, 0,floor(no_of_direction_vectors-3)/2)];
theta_vector = linspace(0, 2*pi, no_of_direction_vectors+1);
theta_vector = theta_vector(1:end-1);
set_of_direction_vectors = [cos(theta_vector); 
                            sin(theta_vector);
                            zeros(2, length(theta_vector))];

%% Definition of the affine hull
affine_hull_of_interest_2D_A = [zeros(2) eye(2)];
affine_hull_of_interest_2D_b = slice_at_vx_vy;
affine_hull_of_interest_2D = Polyhedron('He',...
                                        [affine_hull_of_interest_2D_A,...
                                         affine_hull_of_interest_2D_b]);
init_safe_set = safe_set.intersect(affine_hull_of_interest_2D);

%% Computation of an underapproximative stochastic reach-avoid set
timer_polytope = tic;
[underapproximate_stochastic_reach_avoid_polytope,...
optimal_input_vector_at_boundary_points,...
xmax,...
optimal_input_vector_for_xmax,...
maximum_underapproximate_reach_avoid_probability,...
optimal_theta_i,...
optimal_reachAvoid_i,...
R,...
vertex_poly] = getUnderapproxStochReachAvoidSet(...
    sys, ...
    target_tube, ...
    init_safe_set, ...
    probability_threshold_of_interest, ...
    set_of_direction_vectors);
elapsed_time_polytope = toc(timer_polytope);
fprintf('Time taken for computing the polytope: %1.3f s\n', elapsed_time_polytope);

%% Plot the underapproximative polytope along with the safe and the target sets.
% Assuming only vx-vy is fixed
if ~isEmptySet(underapproximate_stochastic_reach_avoid_polytope)
    underapproximate_stochastic_reach_avoid_polytope_2D =...
        Polyhedron('V',vertex_poly(1:2,:)');
end 
figure(100);
hold on;
plot(safe_set.slice([3,4], slice_at_vx_vy), 'color', 'y');
plot(target_set.slice([3,4], slice_at_vx_vy), 'color', 'k');

scatter(xmax(1), xmax(2), 100,'gs','filled')
if ~isEmptySet(underapproximate_stochastic_reach_avoid_polytope)
    plot(underapproximate_stochastic_reach_avoid_polytope_2D,...
         'color','m','alpha',0.5);
    leg=legend({'Safe set',...
            'Target set',...
            '$\bar{x}_\mathrm{max}$',...
            'Underapprox. set'});
else
    leg=legend({'Safe set','Target set', '$\bar{x}_\mathrm{max}$'});
end
set(leg,'Location','South','Orientation','Horizontal','interpreter','latex');
xlabel('x')
ylabel('y')
axis equal;
box on;
grid on;
axis equal
set(gca,'FontSize',40)
if figure4
    axis([-2.5 2.5 -2 0]);
else
    axis([-1 -0.8 -1.2 -0.9])
    set(leg,'Orientation','Vertical');
end
savefig(gcf,strcat(['MATLAB_figs/CWH_example_',figsave_str,'_initial_vel.fig']),'compact')
saveas(gcf,strcat(['MATLAB_figs/CWH_example_',figsave_str,'_initial_vel.png']))

% title(sprintf('Open-loop underapproximative\nstochastic reach-avoid set'));

%% Plotting a specific one again
direction_index = 11;
figure(201)
clf
hold on;
h1=plot(safe_set.slice([3,4], slice_at_vx_vy), 'color', 'y');
h2=plot(target_set.slice([3,4], slice_at_vx_vy), 'color', 'k');
h3=scatter(vertex_poly(1,direction_index),...
        vertex_poly(2,direction_index),...
        200,'cs','filled');
h4=plot(underapproximate_stochastic_reach_avoid_polytope_2D,...
     'color','m','alpha',0.5);
legend_cell = {'Initial state'};
concat_state_realization = generateMonteCarloSims(...
    n_mcarlo_sims,...
    sys,...
    vertex_poly(:,direction_index),...
    time_horizon,...
    optimal_input_vector_at_boundary_points(:,direction_index));
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
        markerString = 'wd-';
    end
    % Create [x(t_1) x(t_2)... x(t_N)]
    reshaped_X_vector = reshape(...
        concat_state_realization(:,realization_index),sys.state_dim,[]);
    % This realization is to be plotted
    if strcmp(markerString,'g^-')
        if green_legend_updated
            h = plot([vertex_poly(1,direction_index),reshaped_X_vector(1,:)],...
             [vertex_poly(2,direction_index),reshaped_X_vector(2,:)],...
             markerString, 'MarkerSize',10);
        else
            green_legend_updated = 1;
            h5 = plot([vertex_poly(1,direction_index),reshaped_X_vector(1,:)],...
             [vertex_poly(2,direction_index),reshaped_X_vector(2,:)],...
             markerString, 'MarkerSize',10);    
            legend_cell{end+1} = 'Good trajectory';
        end
    elseif strcmp(markerString,'wd-')
        if red_legend_updated
            h = plot([vertex_poly(1,direction_index),reshaped_X_vector(1,:)],...
             [vertex_poly(2,direction_index),reshaped_X_vector(2,:)],...
             markerString, 'MarkerSize',10,'MarkerEdgeColor','k', 'MarkerFaceColor','w');
        else
            h6 = plot([vertex_poly(1,direction_index),reshaped_X_vector(1,:)],...
             [vertex_poly(2,direction_index),reshaped_X_vector(2,:)],...
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
                             vertex_poly(:,direction_index),...
                             time_horizon);
optimal_mean_X = mean_X_sans_input + H_matrix *...
            optimal_input_vector_at_boundary_points(:, direction_index);
optimal_mean_trajectory=reshape(optimal_mean_X,sys.state_dim,[]);
% Plot the optimal mean trajectory from the vertex under study
h7 = scatter(...
      [vertex_poly(1,direction_index), optimal_mean_trajectory(1,:)],...
      [vertex_poly(2,direction_index), optimal_mean_trajectory(2,:)],...
      30, 'bo', 'filled');
legend_cell{end+1} = 'Mean trajectory';
if red_legend_updated
    leg = legend([h3,h5,h6,h7], legend_cell{:},'Location','NorthEast','interpreter','latex');
else
    leg = legend([h3,h5,h7], legend_cell{:},'Location','NorthEast','interpreter','latex');
end
box on;
grid on;
axis equal
xlabel('x')
ylabel('y')
set(gca,'FontSize',20)
fprintf(['Open-loop-based lower bound and Monte-Carlo simulation ',...
                 '(%1.0e particles): %1.3f, %1.3f\n'],...
                n_mcarlo_sims,...
                optimal_reachAvoid_i(direction_index),...
                sum(mcarlo_result)/n_mcarlo_sims);
%% Save fig
savefig(gcf,strcat(['MATLAB_figs/CWH_example_',figsave_str,'_initial_vel_MCarlo.fig']),'compact')
saveas(gcf,strcat(['MATLAB_figs/CWH_example_',figsave_str,'_initial_vel_MCarlo.png']))

