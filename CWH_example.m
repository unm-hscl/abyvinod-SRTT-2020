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

figure3 = 1;  % Setting this to one will do initial velocity of zero
              % Setting this to zero will do initial velocity of 0.01
savefig_true = 0;

%% Dynamics model for the deputy relative to the chief spacecraft
if figure3
    umax=0.1;
    figsave_str = 'zero';
    direction_index_to_plot = 21;
else
    umax = 0.01;
    figsave_str = 'nonzero';
    direction_index_to_plot = 11;
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
no_of_direction_vectors_ccc = 64;  % Increase for a tighter polytopic representation 
                                   % at the cost of higher computation time
no_of_direction_vectors_ft = 8;  % Increase for a tighter polytopic representation 
                                 % at the cost of higher computation time
skip_direction_vectors = 2;    % Skip every x direction vectors for genzps                              
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


if figure3
    % OPTION 1: Initial velocity is zero
    slice_at_vx_vy = zeros(2,1);                 % Initial velocities of interest
else
    % OPTION 2: Initial velocity is non-zero
    slice_at_vx_vy = 0.01*ones(2,1);             % The initial velocities of interest
end
% Construct the set of direction vectors
theta_vector_ccc = linspace(0, 2*pi, no_of_direction_vectors_ccc+1);
theta_vector_ccc = theta_vector_ccc(1:end-1);
set_of_direction_vectors_ccc = [cos(theta_vector_ccc); 
                                sin(theta_vector_ccc);
                                zeros(2, length(theta_vector_ccc))];
theta_vector_ft = linspace(0, 2*pi, no_of_direction_vectors_ft+1);
theta_vector_ft = theta_vector_ft(1:end-1);
set_of_direction_vectors_ft = [cos(theta_vector_ft); 
                               sin(theta_vector_ft);
                               zeros(2, length(theta_vector_ft))];
%% Definition of the affine hull
affine_hull_of_interest_2D_A = [zeros(2) eye(2)];
affine_hull_of_interest_2D_b = slice_at_vx_vy;
affine_hull_of_interest_2D = Polyhedron('He',...
                                        [affine_hull_of_interest_2D_A,...
                                         affine_hull_of_interest_2D_b]);

%% Computation of an underapproximative stochastic reach-avoid set
disp('Chance constrained approach');
timer_polytope_ccc = tic;
[underapproximate_stochastic_reach_avoid_polytope_ccc,...
 optimal_input_vector_at_boundary_points_ccc,...
 xmax_ccc,...
 optimal_input_vector_for_xmax_ccc,...
 maximum_underapproximate_reach_avoid_probability_ccc,...
 optimal_theta_i_ccc,...
 optimal_reachAvoid_i_ccc,...
 vertex_poly_ccc,...
 R] = getUnderapproxStochReachAvoidSet(...
    sys, ...
    target_tube, ...
    affine_hull_of_interest_2D.He, ...
    probability_threshold_of_interest, ...
    set_of_direction_vectors_ccc,...
    'ccc');
elapsed_time_polytope_ccc = toc(timer_polytope_ccc);
fprintf('Time taken for computing the polytope (CCC): %1.3f s\n', elapsed_time_polytope_ccc);
% underapproximate_stochastic_reach_avoid_polytope_ccc = Polyhedron();
disp('Fourier transform approach');
timer_polytope_genzps = tic;
[underapproximate_stochastic_reach_avoid_polytope_ft,...
 optimal_input_vector_at_boundary_points_ft,...
 xmax_ft,...
 optimal_input_vector_for_xmax_ft,...
 maximum_underapproximate_reach_avoid_probability_ft,...
 optimal_theta_i_ft,...
 optimal_reachAvoid_i_ft,...
 vertex_poly_ft] = getUnderapproxStochReachAvoidSet(...
    sys, ...
    target_tube, ...
    affine_hull_of_interest_2D.He, ...
    probability_threshold_of_interest, ...
    set_of_direction_vectors_ft,...
    'genzps');
elapsed_time_polytope_genzps = toc(timer_polytope_genzps);
fprintf('Time taken for computing the polytope (Genz+PS): %1.3f s\n', elapsed_time_polytope_genzps);
% underapproximate_stochastic_reach_avoid_polytope_ft = Polyhedron();

%% Construct the 2D polyhedra
if ~isEmptySet(underapproximate_stochastic_reach_avoid_polytope_ccc)
    underapproximate_stochastic_reach_avoid_polytope_2D_ccc =...
        Polyhedron('V',vertex_poly_ccc(1:2,:)');
else
    underapproximate_stochastic_reach_avoid_polytope_2D_ccc = Polyhedron();
end
if ~isEmptySet(underapproximate_stochastic_reach_avoid_polytope_ft)
    underapproximate_stochastic_reach_avoid_polytope_2D_ft =...
        Polyhedron('V',vertex_poly_ft(1:2,:)');
else
    underapproximate_stochastic_reach_avoid_polytope_2D_ft = Polyhedron();
end

%% Plot the underapproximative polytope along with the safe and the target sets.
% Assuming only vx-vy is fixed
figure(100);
clf
hold on;
plot(safe_set.slice([3,4], slice_at_vx_vy), 'color', 'y');
plot(target_set.slice([3,4], slice_at_vx_vy), 'color', 'k');

scatter(xmax_ccc(1), xmax_ccc(2), 100,'gs','filled')
plot(underapproximate_stochastic_reach_avoid_polytope_2D_ccc,...
     'color','m','alpha',1);
plot(underapproximate_stochastic_reach_avoid_polytope_2D_ft,...
     'color','b','alpha',0.8);
leg=legend({'Safe set',...
        'Target set',...
        '$\bar{x}_\mathrm{max}$',...
        'Chance-constraint',...
        'Fourier transform'});
set(leg,'Location','South','interpreter','latex');
xlabel('x')
ylabel('y')
axis equal;
box on;
grid on;
axis equal
set(gca,'FontSize',20)
if figure3
    axis([-2.5 2.5 -2 0]);
else
    axis([-1 -0.8 -1.2 -0.9])
    set(leg,'Orientation','Vertical');
end
if savefig_true
    savefig(gcf,strcat(['MATLAB_figs/CWH_example_',figsave_str,'_initial_vel.fig']),'compact')
    saveas(gcf,strcat(['MATLAB_figs/CWH_example_',figsave_str,'_initial_vel.png']))
end
% load('D:/Dropbox/MatFiles/2018HSCC/Lesser_CCC_original.mat','x01','x02','Prob','timeSpent')
% caxis([0 1])
% colormap_matrix = colormap('copper');
% color_step_size = 1/(size(colormap_matrix,1)-1);
% colorpolytope = colormap_matrix(round((0.8-.3)/color_step_size)+1,:);
% [~,donotplot_handle]=contourf(x01,x02,Prob-.3,[.8-.3 .8-.3]);
% set(get(get(donotplot_handle,'Annotation'),'LegendInformation'),'IconDisplayStyle','off'); % Exclude line from legend
% plot(Polyhedron('lb',-ones(2,1),'ub',ones(2,1))*0.0001+[0;0.05], 'alpha',1,'color', colorpolytope,'LineStyle','-','LineWidth',0.001);
% plot(underapproximate_stochastic_reach_avoid_polytope_2D_ccc,...
% 'color','m','alpha',0.5);
% title(sprintf('Open-loop underapproximative\nstochastic reach-avoid set'));

%% Plotting a specific one again
init_state = vertex_poly_ccc(:,direction_index_to_plot);
input_vec = optimal_input_vector_at_boundary_points_ccc(:,direction_index_to_plot);
polyt = underapproximate_stochastic_reach_avoid_polytope_2D_ccc;
opt_reach_avoid = optimal_reachAvoid_i_ccc(direction_index_to_plot);
% init_state = vertex_poly_ft(:,direction_index_to_plot);
% input_vec = optimal_input_vector_at_boundary_points_ft(:,direction_index_to_plot);
% polyt = underapproximate_stochastic_reach_avoid_polytope_2D_ft;
% opt_reach_avoid = optimal_reachAvoid_i_ft(direction_index_to_plot);

figure(200+direction_index_to_plot)
clf
hold on;
h1=plot(safe_set.slice([3,4], slice_at_vx_vy), 'color', 'y');
h2=plot(target_set.slice([3,4], slice_at_vx_vy), 'color', 'k');
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
        else
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
optimal_mean_X = mean_X_sans_input + H_matrix * input_vec;
optimal_mean_trajectory=reshape(optimal_mean_X,sys.state_dim,[]);
% Plot the optimal mean trajectory from the vertex under study
h7 = scatter(...
      [init_state(1), optimal_mean_trajectory(1,:)],...
      [init_state(2), optimal_mean_trajectory(2,:)],...
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
                opt_reach_avoid,...
                sum(mcarlo_result)/n_mcarlo_sims);
%% Save fig
if savefig_true
    savefig(gcf,strcat(['MATLAB_figs/CWH_example_',figsave_str,'_initial_vel_MCarlo.fig']),'compact')
    saveas(gcf,strcat(['MATLAB_figs/CWH_example_',figsave_str,'_initial_vel_MCarlo.png']))
end

%theta_vector = [pi/4, pi/2, 3*pi/4,...
                %linspace(pi, 5/4*pi, floor(no_of_direction_vectors-3)/2),...
                %-pi/2 , linspace(-pi/4, 0,floor(no_of_direction_vectors-3)/2)];
