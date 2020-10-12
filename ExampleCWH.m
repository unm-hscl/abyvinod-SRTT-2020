clear;clc;close all

savemat_flag = 1;

problem_setup = 'top';  % 'top'     --- initial velocity of zero
                        % 'bottom'  --- initial velocity of 0.01

%% Dynamics model for the deputy relative to the chief spacecraft
if strcmpi(problem_setup, 'top')
    % OPTION 1: Initial velocity is zero
    slice_at_vx_vy = zeros(2,1);                 % Initial velocities of interest
    umax=0.1;
    figsave_str = 'zero';
    disp('Running zero initial velocity case');
    save_mat_file_path = strcat('./matfiles/','CWH_example_',datestr(now,'YYmmDD_HHMMSS'),'_zero_vel.mat');
elseif strcmpi(problem_setup, 'bottom')
    % OPTION 2: Initial velocity is non-zero
    slice_at_vx_vy = 0.01*ones(2,1);             % The initial velocities of interest
    umax = 0.01;
    figsave_str = 'nonzero';
    disp('Running non-zero initial velocity case');
    save_mat_file_path = strcat('./matfiles/','CWH_example_',datestr(now,'YYmmDD_HHMMSS'),'_nonzero_vel.mat');
else
    throw(SrtInvalidArgsError('problem_setup must be top or bottom'));
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
prob_thresh = 0.8;       % Probability threshold of interest
n_dir_vecs_ccc = 32;     % Increase for a tighter polytopic representation 
                         % at the cost of higher computation time
n_dir_vecs_ft = 32;      % Increase for a tighter polytopic representation 
                         % at the cost of higher computation time
compute_style_ccc = 'cheby';
info_indx = 2;

%% Target set and safe set creation
% Safe set definition --- LoS cone |x|<=-y and y\in[ymin,0] and |vx|<=vxmax and
% |vy|<=vymax
ymin=2;
vxmax=0.5;
vymax=0.5;
A_safe_set = [1, 1, 0, 0;           % x + y <= 0
             -1, 1, 0, 0;           % -x + y <= 0
              0, -1, 0, 0;          % -y <= ymax
              0, 0, 1,0;
              0, 0,-1,0;
              0, 0, 0,1;
              0, 0, 0,-1];
b_safe_set = [0;
              0;
              ymin;
              vxmax;
              vxmax;
              vymax;
              vymax];
safe_set = Polyhedron(A_safe_set, b_safe_set);

% Target set --- Box [-0.1,0.1]x[-0.1,0]x[-0.01,0.01]x[-0.01,0.01]
target_set = Polyhedron('lb', [-0.1; -0.1; -0.01; -0.01],...
                        'ub', [ 0.1;    0;  0.01;  0.01]);
% Target tube definition
target_tube = Tube('reach-avoid', safe_set, target_set, time_horizon);

% Construct the set of direction vectors
theta_vector_ccc = linspace(0, 2*pi, n_dir_vecs_ccc+1);
theta_vector_ccc = theta_vector_ccc(1:end-1);
set_of_direction_vectors_ccc = [cos(theta_vector_ccc); 
                                sin(theta_vector_ccc);
                                zeros(2, length(theta_vector_ccc))];
theta_vector_ft = linspace(0, 2*pi, n_dir_vecs_ft+1);
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
disp(' ')
disp('Chance constrained approach');
timer_polytope_ccc = tic;
cc_options = SReachSetOptions('term', 'chance-open', 'set_of_dir_vecs', ...
        set_of_direction_vectors_ccc, 'init_safe_set_affine', ...
        affine_hull_of_interest_2D, 'verbose', 0, 'compute_style', ...
        compute_style_ccc);
[underapproximate_stochastic_reach_avoid_polytope_ccc, extra_info_ccc] ...
    = SReachSet('term','chance-open', sys, prob_thresh, target_tube, ...
        cc_options);      
elapsed_time_polytope_ccc = toc(timer_polytope_ccc);
fprintf('Time taken for computing the polytope (CCC): %1.3f s\n', ...
    elapsed_time_polytope_ccc);

%% Fourier transform
disp(' ')
disp('Fourier transform approach');
timer_polytope_genzps = tic;
genzps_options = SReachSetOptions('term', 'genzps-open', 'set_of_dir_vecs', ...
        set_of_direction_vectors_ft, 'init_safe_set_affine', ...
        affine_hull_of_interest_2D, 'verbose', 0, 'desired_accuracy', 1e-2);
        %, ... 'PSoptions', psoptimset('Display','iter'));
[underapproximate_stochastic_reach_avoid_polytope_ft, extra_info_ft] ...
    = SReachSet('term','genzps-open', sys, prob_thresh, target_tube, ...
        genzps_options);      
elapsed_time_polytope_genzps = toc(timer_polytope_genzps);
fprintf('Time taken for computing the polytope (Genz+PS): %1.3f s\n', ...
    elapsed_time_polytope_genzps);

%% Construct the 2D polyhedra
if ~isEmptySet(underapproximate_stochastic_reach_avoid_polytope_ccc)
    underapproximate_stochastic_reach_avoid_polytope_2D_ccc =...
        Polyhedron('V', ...
        underapproximate_stochastic_reach_avoid_polytope_ccc.V(:,1:2));
else
    underapproximate_stochastic_reach_avoid_polytope_2D_ccc = Polyhedron();
end
if ~isEmptySet(underapproximate_stochastic_reach_avoid_polytope_ft)
    underapproximate_stochastic_reach_avoid_polytope_2D_ft =...
        Polyhedron('V', ...
        underapproximate_stochastic_reach_avoid_polytope_ft.V(:,1:2));
else
    underapproximate_stochastic_reach_avoid_polytope_2D_ft = Polyhedron();
end

%% Validation via Monte-Carlo sims
fprintf('\nValidation via Monte-Carlo simulation\n');
n_mcarlo_sims = 1e5;

% Chance constraint validation
mcarlo_prob_ccc = zeros(n_dir_vecs_ccc, 1);
for dir_indx = 1:n_dir_vecs_ccc
    fprintf('Vertex number (CCC) %2d: ', dir_indx);    
    initial_state = extra_info_ccc(info_indx).vertices_underapprox_polytope(:,dir_indx);
    opt_open_ctr = extra_info_ccc(info_indx).opt_input_vec_at_vertices(:,dir_indx);    
    mcarlo_prob_ccc(dir_indx) = simProb(n_mcarlo_sims, sys, initial_state, ...
        time_horizon, opt_open_ctr, target_tube);
end

% Fourier transform validation
mcarlo_prob_ft = zeros(n_dir_vecs_ft, 1);
for dir_indx = 1:n_dir_vecs_ft
    fprintf('Vertex number (FT) %2d: ', dir_indx);    
    initial_state = extra_info_ft.vertices_underapprox_polytope(:,dir_indx);
    opt_open_ctr = extra_info_ft.opt_input_vec_at_vertices(:,dir_indx);    
    mcarlo_prob_ft(dir_indx) = simProb(n_mcarlo_sims, sys, initial_state, ...
        time_horizon, opt_open_ctr, target_tube);
end
% Save data
if savemat_flag
    save(save_mat_file_path);
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

%% Function for simulation
function mc_prob = simProb(n_mcarlo_sims, sys, initial_state, time_horizon, ...
    opt_open_ctr, target_tube)
    % Generate Monte-Carlo simulations using the srlcontrol and
    % generateMonteCarloSims
    [X,~,~] = generateMonteCarloSims(n_mcarlo_sims, sys, initial_state, ...
        time_horizon, opt_open_ctr);
    mc_prob = sum(target_tube.contains(X))/n_mcarlo_sims;
    fprintf('Simulated probability: %1.3f\n', mc_prob)
end