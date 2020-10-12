% This code took a total of 35 minutes to construct the stochastic reach sets
% for a LTI system with dimension varying from 2 to 10.
clear;clc;close all;

compute_style_ccc = 'cheby';  %Options --- 'all','max_safe_init','cheby'
prob_thresh = 0.8;
dist_cov_list = [5e-2, 0.1, 0.2, 0.5];
time_horizon = 10;
safety_set_bound = 3;

%% Computation of an underapproximative stochastic reach-avoid set
verbose_cc = 0;
sys_state_dim = 2;
no_of_direction_vectors_ccc = 8;
theta_vector_ccc = linspace(0, 2*pi, no_of_direction_vectors_ccc+1);
theta_vector_ccc = theta_vector_ccc(1:end-1);
set_of_direction_vectors_ccc = [cos(theta_vector_ccc); 
                                sin(theta_vector_ccc)];

%% Setup the target tube
xmax = safety_set_bound * ones(sys_state_dim,1);
% safe set definition
safe_set = Polyhedron('lb', -xmax, 'ub', xmax);
% target tube definition
target_tube = Tube('viability', safe_set, time_horizon);

cc_options = SReachSetOptions('term', 'chance-open', ...
    'set_of_dir_vecs', set_of_direction_vectors_ccc, ...
    'init_safe_set_affine', Polyhedron(), 'verbose', verbose_cc, ...
    'compute_style', compute_style_ccc);

% For each of the dimension
volume_matrix = zeros(length(dist_cov_list),1);
elapsed_time_matrix = zeros(length(dist_cov_list),1);
for index = 1:length(dist_cov_list)
    dist_cov = dist_cov_list(index);
    dist = RandomVector('Gaussian', zeros(sys_state_dim,1), ...
        dist_cov * eye(sys_state_dim));
    sys = LtiSystem('StateMatrix', 0.8 * eye(sys_state_dim), ...
                    'DisturbanceMatrix', eye(sys_state_dim), ...
                    'Disturbance', dist);

    timer_polytope_ccc = tic;
    [underapproximate_stochastic_reach_avoid_polytope_ccc, ...
        extra_info_cell] = SReachSet(...
        'term','chance-open', sys, prob_thresh, target_tube, cc_options);  
    elapsed_time_matrix(index) = toc(timer_polytope_ccc);

    volume_matrix(index) = ...
        underapproximate_stochastic_reach_avoid_polytope_ccc.volume;
    fprintf(['Dim: %d | Variance: %1.2f | Volume: %1.2f | ', ...
        'Elapsed time: %1.4f\n'], sys_state_dim, dist_cov, ...
        volume_matrix(index), elapsed_time_matrix(index));
end
disp(volume_matrix);
disp(elapsed_time_matrix);