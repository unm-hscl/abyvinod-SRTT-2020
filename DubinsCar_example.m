clear
close all
clc

dropboxpath='D:/Dropbox';
fontSize=20;
% dropboxpath='/datafiles/Dropbox';
%fontSize=40;
save_mat_file_path = strcat(dropboxpath,'/MatFiles/2018TAC_Verification/','DubinsCar_example_',datestr(now,'YYYYmmDD_HHMMSS'),'.mat');

time_horizon = 50;
time_const = 1/2*time_horizon;
init_heading = pi/10;
sampling_time = 0.1;
box_halflength = 4;
omega = pi/time_horizon/sampling_time;
turning_rate = omega*ones(time_horizon,1);
dist_cov = 0.001;
probability_threshold_of_interest = 0.8;
no_of_direction_vectors_ccc = 16;
no_of_direction_vectors_ft = 8;
direction_index_to_plot = 2;
n_mcarlo_sims = 1e5;
n_sims_to_plot = 10;
axis_vec = [-8    10   -5   21];
v_nominal = 10;
umax = v_nominal/3*2;

%% LTV system definition
[sys, heading_vec] = getDubinsCarLtv('add-dist',...
turning_rate,...
init_heading,...
sampling_time,...
Polyhedron('lb',0,'ub',umax),...
eye(2),...
RandomVector('Gaussian',zeros(2,1), dist_cov * eye(2)));

target_tube_cell = cell(time_horizon + 1,1);

%% Target tube definition
% figure(100);clf;hold on
angle_at_the_center = (heading_vec) - pi/2;
center_box = zeros(2, time_horizon + 1);        
for itt=0:time_horizon
    center_box(:, itt+1) = v_nominal * [cos(angle_at_the_center(itt+1))-cos(angle_at_the_center(1));
                                        sin(angle_at_the_center(itt+1))-sin(angle_at_the_center(1))];
    target_tube_cell{itt+1} = Polyhedron('lb',center_box(:, itt+1) - box_halflength * exp(- itt/time_const), 'ub', center_box(:, itt+1) + box_halflength*exp(- itt/time_const));
%     plot(target_tube_cell{itt+1},'alpha',0.5);
end
% axis equal
% axis(axis_vec)
target_tube = TargetTube(target_tube_cell{:});

%% Set of direction vectors
theta_vector_ccc = linspace(0, 2*pi, no_of_direction_vectors_ccc+1);
theta_vector_ccc = theta_vector_ccc(1:end-1);
set_of_direction_vectors_ccc = [cos(theta_vector_ccc); 
                                sin(theta_vector_ccc)];
% theta_vector_ft = linspace(0, 2*pi, no_of_direction_vectors_ft+1);
% theta_vector_ft = theta_vector_ft(1:end-1);
% set_of_direction_vectors_ft = [cos(theta_vector_ft); 
%                                sin(theta_vector_ft)];
%% Set computation                           
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
% disp('Fourier transform approach');
% timer_polytope_genzps = tic;
% [underapproximate_stochastic_reach_avoid_polytope_ft,...
%  optimal_input_vector_at_boundary_points_ft,...
%  xmax_ft,...
%  optimal_input_vector_for_xmax_ft,...
%  maximum_underapproximate_reach_avoid_probability_ft,...
%  optimal_theta_i_ft,...
%  optimal_reachAvoid_i_ft,...
%  vertex_poly_ft] = getUnderapproxStochReachAvoidSet(...
%     sys, ...
%     target_tube, ...
%     [], ...
%     probability_threshold_of_interest, ...
%     set_of_direction_vectors_ft,...
%     'genzps');
% elapsed_time_polytope_genzps = toc(timer_polytope_genzps);
% fprintf('Time taken for computing the polytope (Genz+PS): %1.3f s\n', elapsed_time_polytope_genzps);

%% Save the data
save(save_mat_file_path);

%% Plot the set
figure(101);
clf;
hold on;
plot(target_tube(1));
% plot(underapproximate_stochastic_reach_avoid_polytope_ccc,'color','m');
plot(underapproximate_stochastic_reach_avoid_polytope_ft,'color','b');
axis equal
axis (1.2*[-box_halflength box_halflength -box_halflength box_halflength]);
box on;
legend('Target set at t=0','Stochastic reach set','Location','SouthEast');
