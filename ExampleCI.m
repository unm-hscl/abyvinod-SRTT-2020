clear;clc;close all;

save_mat_file_path = sprintf('./matfiles/CI_example_%s.mat', ...
        datestr(now,'YYmmDD_HHMMSS'));
savemat_flag = 1;

compute_style_ccc = 'cheby';  %Options --- 'all','max_safe_init','cheby'
alpha_vec = [0.6 0.85 0.9];
no_of_direction_vectors_ccc = 8;

%% System definition
n_intg = 40;
umax = 1;
dist_cov = 0.01;
sampling_time = 0.1;                           
sys = getChainOfIntegLtiSystem(n_intg, sampling_time, ...
    Polyhedron('lb',-umax,'ub',umax), ...
    RandomVector('Gaussian', zeros(n_intg,1), dist_cov * eye(n_intg)));

%% Setup the target tube
% safe set definition
xmax_safe = 10;
xmax_target = 8;
safe_set = Polyhedron('lb', -xmax_safe*ones(n_intg,1), 'ub', xmax_safe*ones(n_intg,1));
target_set = Polyhedron('lb', -xmax_target*ones(n_intg,1), 'ub', xmax_target*ones(n_intg,1));
% target tube definition
time_horizon = 5;
target_tube = Tube('reach-avoid', safe_set, target_set, time_horizon);

%% Computation of an underapproximative stochastic reach-avoid set
n_zeroed = 2;
if n_zeroed == 2
    theta_vector_ccc = linspace(0, 2*pi, no_of_direction_vectors_ccc+1);
    theta_vector_ccc = theta_vector_ccc(1:end-1);
    set_of_direction_vectors_ccc = ...
                        [cos(theta_vector_ccc); 
                         sin(theta_vector_ccc);
                         zeros(n_intg - n_zeroed, no_of_direction_vectors_ccc)];
elseif n_zeroed == 3
    theta_vector_ccc = linspace(0, 2*pi, sqrt(no_of_direction_vectors_ccc)+1);
    theta_vector_ccc = theta_vector_ccc(1:end-1);
    scaling_vector_ccc = linspace(0, 1, sqrt(no_of_direction_vectors_ccc)+1);
    set_of_direction_vectors_ccc = [];
    for theta_i = 1: sqrt(no_of_direction_vectors_ccc)
            v = scaling_vector_ccc.*[cos(theta_vector_ccc(theta_i)); 
                                     sin(theta_vector_ccc(theta_i))];
            vz = sqrt(1 - scaling_vector_ccc);
            new_vectors = [v,v;vz,-vz];
            set_of_direction_vectors_ccc = [set_of_direction_vectors_ccc, ...
                new_vectors];
    end
    no_of_direction_vectors_ccc = length(set_of_direction_vectors_ccc);
    set_of_direction_vectors_ccc = ...
                        [set_of_direction_vectors_ccc;
                         zeros(n_intg - n_zeroed, no_of_direction_vectors_ccc)];
end

affine_hull_He = [zeros(n_intg - n_zeroed, n_zeroed), ...
                  eye(n_intg -n_zeroed), ...
                  zeros(n_intg - n_zeroed, 1)];
init_safe_set_affine_poly = Polyhedron('He', affine_hull_He);

elapsed_time_polytope_ccc = zeros(length(alpha_vec),1);
underapproximate_stochastic_reach_avoid_polytope_ccc = repmat(Polyhedron(), ...
    length(alpha_vec),1);
for indx=1:length(alpha_vec)
    prob_thresh = alpha_vec(indx);
    fprintf('Chance constrained approach for alpha=%1.2f\n',prob_thresh);
    timer_polytope_ccc = tic;
    cc_options = SReachSetOptions('term', 'chance-open', 'set_of_dir_vecs', ...
        set_of_direction_vectors_ccc, 'init_safe_set_affine', ...
        init_safe_set_affine_poly, 'verbose', 0, 'compute_style', ...
        compute_style_ccc);
    [underapproximate_stochastic_reach_avoid_polytope_ccc(indx), extra_info] ...
        = SReachSet('term','chance-open', sys, prob_thresh, target_tube, ...
            cc_options);      
    elapsed_time_polytope_ccc(indx) = toc(timer_polytope_ccc);    
end

%% OL interpolation
timer_interp = tic;
interp_set = interpViaMinkSum(alpha_vec(2), ...
    underapproximate_stochastic_reach_avoid_polytope_ccc(3), alpha_vec(3),...
    underapproximate_stochastic_reach_avoid_polytope_ccc(1), alpha_vec(1));
elapsed_time_interp = toc(timer_interp);

%% Save all the data
if savemat_flag
    save(save_mat_file_path)
end