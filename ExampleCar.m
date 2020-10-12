clear
close all
clc

fontSize=20;
save_mat_file_path = strcat('./matfiles/Car_example_',datestr(now,'YYmmDD_HHMMSS'),'.mat');
compute_style_ccc = 'cheby';%'max_safe_init';
info_indx = 2;

delta = 0.7;
prob_thresh = 0.8;                         

%% LTV system definition
sampling_time = 0.1;                        % Sampling time
init_heading = pi/10;                       % Initial heading 
% Known turning rate sequence
time_horizon = 50;
omega = pi/time_horizon/sampling_time;
turning_rate = omega*ones(time_horizon,1);
% Input space definition
umax = 10;
input_space = Polyhedron('lb',0,'ub',umax);
% Disturbance matrix and random vector definition
dist_matrix = eye(2);
eta_dist_gauss = RandomVector('Gaussian',zeros(2,1), 0.001 * eye(2));

sys = getDubinsCarLtv('add-dist', turning_rate, init_heading, ...
    sampling_time, input_space, dist_matrix, eta_dist_gauss);
[~,H,~] = sys.getConcatMats(time_horizon);


%% Target tube definition
box_halflength_at_0 = 4;                % Box half-length at t=0
no_of_direction_vectors_ccc = 16;
time_const = 1/2*time_horizon;          % Time constant characterize the
                                        % exponentially decaying box half-length
v_nominal = umax * delta;               % Nominal trajectory's heading velocity
% Construct the nominal trajectory
center_box_X = [zeros(2,1);
                H * (v_nominal * ones(time_horizon,1))];
center_box = reshape(center_box_X,2,[]);

% Target tube definition as well as plotting
target_tube_cell = cell(time_horizon + 1,1); % Vector to store target sets
for itt = 0:time_horizon
    % Define the target set at time itt
    target_tube_cell{itt+1} = Polyhedron(...
        'lb',center_box(:, itt+1) -box_halflength_at_0*exp(- itt/time_const),...
        'ub', center_box(:, itt+1) + box_halflength_at_0*exp(- itt/time_const));
end
target_tube = Tube(target_tube_cell{:});

%% Convex chance constrained approach
fprintf('Convex chance-constrained approach\n');
% Set of direction vectors
theta_vector_ccc = linspace(0, 2*pi, no_of_direction_vectors_ccc+1);
theta_vector_ccc = theta_vector_ccc(1:end-1);
set_of_direction_vectors_ccc = [cos(theta_vector_ccc); 
                                sin(theta_vector_ccc)];
timer_polytope_ccc = tic;
opts = SReachSetOptions('term', 'chance-open', 'pwa_accuracy', 1e-3, ...
        'set_of_dir_vecs', set_of_direction_vectors_ccc, ...
        'init_safe_set_affine',Polyhedron(),'verbose', 0, ...
        'compute_style', compute_style_ccc);
[ccc_polytope, extra_info] = SReachSet('term','chance-open', sys, ...
      prob_thresh, target_tube, opts);
elapsed_time_polytope_ccc = toc(timer_polytope_ccc);
fprintf('Time taken for computing the polytope (CCC): %1.3f s\n', ...
    elapsed_time_polytope_ccc);

%% Validation via Monte-Carlo sims
fprintf('\nValidation via Monte-Carlo simulation\n');
n_mcarlo_sims = 1e5;
dir_indx_test = 5;
mcarlo_prob = zeros(no_of_direction_vectors_ccc, 1);
for dir_indx = 1:no_of_direction_vectors_ccc
    fprintf('Vertex number %2d: ', dir_indx);
    initial_state = extra_info(info_indx).vertices_underapprox_polytope(:,dir_indx);
    opt_open_ctr = extra_info(info_indx).opt_input_vec_at_vertices(:,dir_indx);
    opt_lb = extra_info(info_indx).opt_reach_prob_i(dir_indx);
    % Generate Monte-Carlo simulations using the srlcontrol and
    % generateMonteCarloSims
    [X,~,~] = generateMonteCarloSims(n_mcarlo_sims, sys, initial_state, ...
        time_horizon, opt_open_ctr);
    mcarlo_prob(dir_indx) = sum(target_tube.contains(X))/n_mcarlo_sims;
    fprintf('Simulated probability: %1.3f\n', mcarlo_prob(dir_indx))
    if dir_indx == dir_indx_test
        X_test = X;
    end
end

%% Save the data
clear X;                  % Reduce the size
save(save_mat_file_path);