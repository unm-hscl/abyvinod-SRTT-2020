% This code took a total of 35 minutes to construct the stochastic reach sets
% for a LTI system with dimension varying from 2 to 10.
clear;clc;close all;

low_dir_vecs = false;       % Change it to true for 8, and false for 32 vectors
savemat_flag = 1;

compute_style_ccc = 'cheby';  %Options --- 'all','max_safe_init','cheby'
prob_thresh = 0.6;
dist_cov = 5e-2;
time_horizon = 10;
verbose_cc = 0;

%% Computation of an underapproximative stochastic reach-avoid set
sys_state_dim = 2;
if low_dir_vecs
    no_of_direction_vectors_ccc = 8;
else
    no_of_direction_vectors_ccc = 32;
end
theta_vector_ccc = linspace(0, 2*pi, no_of_direction_vectors_ccc+1);
theta_vector_ccc = theta_vector_ccc(1:end-1);
set_of_direction_vectors_ccc = [cos(theta_vector_ccc); 
                                sin(theta_vector_ccc)];
save_mat_file_path = sprintf(...
   './Figure7_data/results_2D_sreachtools/StocHy_ex_%s_2D_no_of_dirs_%d.mat',...
        datestr(now,'YYmmDD_HHMMSS'), no_of_direction_vectors_ccc);


% For each of the dimension
dist = RandomVector('Gaussian', zeros(sys_state_dim,1), ...
    dist_cov * eye(sys_state_dim));
sys = LtiSystem('StateMatrix', 0.8 * eye(sys_state_dim), ...
                'DisturbanceMatrix', eye(sys_state_dim), ...
                'Disturbance', dist);

%% Setup the target tube
xmax = ones(sys_state_dim,1);
% safe set definition
safe_set = Polyhedron('lb', -xmax, 'ub', xmax);
% target tube definition
target_tube = Tube('viability', safe_set, time_horizon);

cc_options = SReachSetOptions('term', 'chance-open', ...
    'set_of_dir_vecs', set_of_direction_vectors_ccc, ...
    'init_safe_set_affine', Polyhedron(), 'verbose', verbose_cc, ...
    'compute_style', compute_style_ccc);
timer_polytope_ccc = tic;
[underapproximate_stochastic_reach_avoid_polytope_ccc, ...
    extra_info_cell] = SReachSet(...
    'term','chance-open', sys, prob_thresh, target_tube, cc_options);  
elapsed_time_polytope_ccc = toc(timer_polytope_ccc);


fprintf('Dim: %d | Number of directions: %d | Elapsed time: %1.4f\n', ...
    sys_state_dim, size(set_of_direction_vectors_ccc, 2), ...
    elapsed_time_polytope_ccc);

disp('X dim')
disp(underapproximate_stochastic_reach_avoid_polytope_ccc.V(:,1));
disp('Y dim')
disp(underapproximate_stochastic_reach_avoid_polytope_ccc.V(:,2));
disp('Copy this over to the Python script')

%% Save all the data
if savemat_flag
    save(save_mat_file_path);
end
