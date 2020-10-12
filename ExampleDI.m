clear;clc;close all;

save_mat_file_path = sprintf('./matfiles/DI_example_%s.mat', ...
        datestr(now,'YYmmDD_HHMMSS'));
savemat_flag = 1;

compute_style_ccc = 'cheby';  %Options --- 'all','max_safe_init','cheby'
alpha_vec = [0.6 0.85 0.9];
no_of_direction_vectors_ccc = 32;

%% System definition
n_intg = 2;
umax = 0.1;
dist_cov = 0.01;
sampling_time = 0.1;                           
sys = getChainOfIntegLtiSystem(n_intg, sampling_time, ...
    Polyhedron('lb',-umax,'ub',umax), ...
    RandomVector('Gaussian', zeros(n_intg,1), dist_cov * eye(n_intg)));

%% Setup the target tube
xmax = [1,1];
% safe set definition
safe_set = Polyhedron('lb', -xmax, 'ub', xmax);
% target tube definition
time_horizon = 10;
target_tube = Tube('viability', safe_set, time_horizon);

% Parameters for dynamic programming and visualization
% ----------------------------------------------------
% Step sizes for gridding
dyn_prog_xinc = 0.05;
dyn_prog_uinc = 0.05;

%% Dynamic programming computation
% Grid-based dynamic programming
disp('Grid-based dynamic programming');
timer_DP=tic;
[prob_x, cell_of_xvecs] = SReachDynProg('term', sys, dyn_prog_xinc, ...
    dyn_prog_uinc, target_tube);
elapsed_time_DP_recursion = toc(timer_DP);
x = cell_of_xvecs{1};
% Set computation
disp('Compute stochastic reach sets from the optimal value functions');
timer_DP_set = tic;
poly_array = getDynProgLevelSets2D(cell_of_xvecs, prob_x, alpha_vec, ...
    target_tube);
elapsed_time_DP_set = toc(timer_DP_set);
elapsed_time_DP_total = elapsed_time_DP_recursion + elapsed_time_DP_set;

%% Computation of an underapproximative stochastic reach-avoid set
theta_vector_ccc = linspace(0, 2*pi, no_of_direction_vectors_ccc+1);
theta_vector_ccc = theta_vector_ccc(1:end-1);
set_of_direction_vectors_ccc = [cos(theta_vector_ccc); 
                                sin(theta_vector_ccc)];

elapsed_time_polytope_ccc = zeros(length(alpha_vec),1);
underapproximate_stochastic_reach_avoid_polytope_ccc = repmat(Polyhedron(), ...
    length(alpha_vec),1);

% For each of the threshold
for indx=1:length(alpha_vec)
    prob_thresh = alpha_vec(indx);
    fprintf('Chance constrained approach for alpha=%1.2f\n', prob_thresh);
    cc_options = SReachSetOptions('term', 'chance-open', ...
        'set_of_dir_vecs', set_of_direction_vectors_ccc, ...
        'init_safe_set_affine', Polyhedron(), 'verbose', 0,  ...
        'compute_style', compute_style_ccc);
    timer_polytope_ccc = tic;
    underapproximate_stochastic_reach_avoid_polytope_ccc(indx) = ...
        SReachSet('term','chance-open', sys, prob_thresh, target_tube, ...
        cc_options);  
    elapsed_time_polytope_ccc(indx) = toc(timer_polytope_ccc);
end

%% DL interpolation
timer_interp_DP=tic;
interp_set_DP = interpViaMinkSum(alpha_vec(2), poly_array(3), alpha_vec(3),...
    poly_array(1), alpha_vec(1));
elapsed_time_interp_DP = toc(timer_interp_DP);

%% OL interpolation
timer_interp = tic;
interp_set = interpViaMinkSum(alpha_vec(2), ...
    underapproximate_stochastic_reach_avoid_polytope_ccc(3), alpha_vec(3),...
    underapproximate_stochastic_reach_avoid_polytope_ccc(1), alpha_vec(1));
elapsed_time_interp = toc(timer_interp);

%% Save all the data
if savemat_flag
    save(save_mat_file_path);
end