% This code took a total of 35 minutes to construct the stochastic reach sets
% for a LTI system with dimension varying from 2 to 10.
clear;clc;close all;

save_mat_file_path = sprintf('./Figure8_data/StocHy_example_%s.mat', ...
        datestr(now,'YYmmDD_HHMMSS'));
savemat_flag = 1;

dist_cov = 5e-2;
time_horizon = 10;
verbose_cc = 0;

% Irrelevant options, since only W_max is computed
compute_style_ccc = 'max_safe_init';  %Options --- 'all','max_safe_init','cheby'
prob_thresh = 0.5;  


%% Computation of an underapproximative stochastic reach-avoid set
sys_state_dim_vec = 2:10;
elapsed_time_polytope_ccc = zeros(length(sys_state_dim_vec),1);
elapsed_time_Wmax_ccc = zeros(length(sys_state_dim_vec),1);
underapproximate_stochastic_reach_avoid_polytope_ccc = repmat(Polyhedron(), ...
    length(sys_state_dim_vec),1);
extra_info_cell = {};

% For each of the dimension
for indx = 1:length(sys_state_dim_vec) 
    sys_state_dim = sys_state_dim_vec(indx);
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

    %% Compute the set
    % 2^n direction vectors as the vertices of the safe set
    set_of_direction_vectors_ccc = safe_set.V';
    cc_options = SReachSetOptions('term', 'chance-open', ...
        'set_of_dir_vecs', set_of_direction_vectors_ccc, ...
        'init_safe_set_affine', Polyhedron(), 'verbose', verbose_cc, ...
        'compute_style', compute_style_ccc);
    timer_polytope_ccc = tic;
    [underapproximate_stochastic_reach_avoid_polytope_ccc(indx), ...
        extra_info_cell{indx}] = SReachSet(...
        'term','chance-open', sys, prob_thresh, target_tube, cc_options);  
    elapsed_time_polytope_ccc(indx) = toc(timer_polytope_ccc);
    
    %% Compute only W_max
    set_of_direction_vectors_ccc = [];
    cc_options = SReachSetOptions('term', 'chance-open', ...
        'set_of_dir_vecs', set_of_direction_vectors_ccc, ...
        'init_safe_set_affine', Polyhedron(), 'verbose', verbose_cc, ...
        'compute_style', compute_style_ccc);
    timer_Wmax_ccc = tic;
    SReachSet('term','chance-open', sys, prob_thresh, target_tube, cc_options);  
    elapsed_time_Wmax_ccc(indx) = toc(timer_Wmax_ccc);
    fprintf('Dim: %d | Elapsed time: Wmax: %1.4f, polytope: %1.4f\n',...
        sys_state_dim, elapsed_time_Wmax_ccc(indx), ...
        elapsed_time_polytope_ccc(indx));
end


%% Save all the data
if savemat_flag
    save(save_mat_file_path);
end