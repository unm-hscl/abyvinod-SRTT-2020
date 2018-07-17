clear
clc
close all

fontSize=40;
sampling_time = 0.25;                           % sampling time
time_horizon = 5;
alpha_vec = [0.9];
no_of_direction_vectors_ccc = 8;

%% System definition
n_zeroed = 2;
n_intg = 40;
umax = 1;
dist_cov = 0.01;
sys = getChainOfIntegLtiSystem(n_intg, sampling_time, Polyhedron('lb',-umax,'ub',umax),...
        RandomVector('Gaussian', zeros(n_intg,1), dist_cov * eye(n_intg)));

%% Setup the target tube
% safe set definition
safe_set = Polyhedron('lb', -10*ones(n_intg,1), 'ub', 10*ones(n_intg,1));
target_set = Polyhedron('lb', -8*ones(n_intg,1), 'ub', 8*ones(n_intg,1));
% target tube definition
target_tube = TargetTube('reach-avoid', safe_set, target_set, time_horizon);

%% Computation of an underapproximative stochastic reach-avoid set
if n_zeroed == 2
    theta_vector_ccc = linspace(0, 2*pi, no_of_direction_vectors_ccc+1);
    theta_vector_ccc = theta_vector_ccc(1:end-1);
    set_of_direction_vectors_ccc = [cos(theta_vector_ccc); 
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
            set_of_direction_vectors_ccc = [set_of_direction_vectors_ccc,new_vectors];
    end
    no_of_direction_vectors_ccc = length(set_of_direction_vectors_ccc);
    set_of_direction_vectors_ccc = [set_of_direction_vectors_ccc;
                                    zeros(n_intg - n_zeroed, no_of_direction_vectors_ccc)];
end

affine_hull_He = [zeros(n_intg - n_zeroed, n_zeroed), eye(n_intg -n_zeroed), zeros(n_intg - n_zeroed, 1)];

i=1;
elapsed_time_polytope_ccc = zeros(length(alpha_vec),1);
underapproximate_stochastic_reach_avoid_polytope_ccc = repmat(Polyhedron(),length(alpha_vec),1);
for prob_thresh_of_interest = alpha_vec
    fprintf('Chance constrained approach for alpha=%1.2f\n',prob_thresh_of_interest);
    timer_polytope_ccc = tic;
    underapproximate_stochastic_reach_avoid_polytope_ccc(i) = getUnderapproxStochReachAvoidSet(...
        sys, ...
        target_tube, ...
        affine_hull_He, ...
        prob_thresh_of_interest, ...
        set_of_direction_vectors_ccc,...
        'ccc');
    elapsed_time_polytope_ccc(i) = toc(timer_polytope_ccc);    
    i = i+1;
end

%% Open-loop underapproximation
figure(12);
clf
hold on
color_string = ['g','b','y'];
for i=1:2:length(alpha_vec)
    plot(underapproximate_stochastic_reach_avoid_polytope_ccc(i).slice(n_zeroed+1:n_intg,zeros(n_intg-n_zeroed,1)),'color',color_string(i),'alpha',(i/3));
end
box on
xlabel('x')
ylabel('y')
zlabel('z')
set(gca,'FontSize',fontSize);

%% OL interpolation
timer_interp = tic;
interp_set = interpStochReachAvoidSet(...
    alpha_vec(2),...
    underapproximate_stochastic_reach_avoid_polytope_ccc(3),...        
    alpha_vec(3),...
    underapproximate_stochastic_reach_avoid_polytope_ccc(1),...        
    alpha_vec(1));
elapsed_time_interp = toc(timer_interp);

%% Save all the data
save('CI_example.mat');
    
%% Interpolation comparison
figure(13)
clf
hold on
plot(underapproximate_stochastic_reach_avoid_polytope_ccc(2).slice(n_zeroed+1:n_intg,zeros(n_intg-n_zeroed,1)),'color','b','alpha',0.3);
plot(interp_set.slice(n_zeroed+1:n_intg,zeros(n_intg-n_zeroed,1)),'color','c','alpha',0.8);
axis equal
box on
xlabel('x')
ylabel('y')
set(gca,'FontSize',fontSize);
