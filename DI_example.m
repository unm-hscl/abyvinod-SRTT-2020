clear
clc

fontSize=40;
T = 0.25;                           % sampling time
time_horizon = 5;
alpha_vec = [0.6 0.8 0.9];
no_of_direction_vectors_ccc = 64;

load_data = 0;

if load_data
    load('DP_grid.mat');
else
    % define the system
    sys = LtiSystem('StateMatrix', [1, T; 0, 1], ...
        'InputMatrix', [T^2/2; T], ...
        'InputSpace', Polyhedron('lb', -0.1, 'ub', 0.1), ...
        'DisturbanceMatrix', eye(2), ...
        'Disturbance', StochasticDisturbance('Gaussian', zeros(2,1), 0.01*eye(2)));
    %% Setup the target tube
    % safe set definition
    safe_set = Polyhedron('lb', [-1, -1], 'ub', [1, 1]);
    % target set definition
    target_set = Polyhedron('lb', [-0.5, -0.5], 'ub', [0.5, 0.5]);
    % target tube definiton
    target_tube_with_tZero = TargetTube('viability', safe_set, time_horizon+1);

    %% Dynamic programming recursion via gridding
    % For dynamic programming, we need to create a grid over which we will perform 
    % the recursion
    %%
    % need to create a state space grid and input space grid
    ss_grid = SpaceGrid([-1, -1], [1, 1], 40);
    in_grid = InputGrid(-1, 1, 20);

    timer_DP=tic;
    grid_probability = getDynProgSolForTargetTube(sys, ...
        ss_grid, in_grid, target_tube_with_tZero);
    elapsed_time_DP = toc(timer_DP);
    
    %% Computation of an underapproximative stochastic reach-avoid set
    target_tube = TargetTube('viability',safe_set, time_horizon);
    init_safe_set = safe_set;
    theta_vector_ccc = linspace(0, 2*pi, no_of_direction_vectors_ccc+1);
    theta_vector_ccc = theta_vector_ccc(1:end-1);
    set_of_direction_vectors_ccc = [cos(theta_vector_ccc); 
                                    sin(theta_vector_ccc)];

    i=1;
    elapsed_time_polytope_ccc = zeros(length(alpha_vec),1);
    underapproximate_stochastic_reach_avoid_polytope_ccc = repmat(Polyhedron(),length(alpha_vec),1);
    for prob_thresh_of_interest = alpha_vec
        fprintf('Chance constrained approach for alpha=%1.2f\n',prob_thresh_of_interest);
        timer_polytope_ccc = tic;
        underapproximate_stochastic_reach_avoid_polytope_ccc(i) = getUnderapproxStochReachAvoidSet(...
            sys, ...
            target_tube, ...
            init_safe_set, ...
            prob_thresh_of_interest, ...
            set_of_direction_vectors_ccc,...
            'ccc');
        elapsed_time_polytope_ccc(i) = toc(timer_polytope_ccc);
        i = i+1;
    end
    save('DP_grid.mat','grid_probability', 'sys', 'ss_grid', 'in_grid', 'target_tube_with_tZero',...
    'underapproximate_stochastic_reach_avoid_polytope_ccc','elapsed_time_polytope_ccc', ...
    'target_tube', 'init_safe_set', 'prob_thresh_of_interest', 'set_of_direction_vectors_ccc',...
    'elapsed_time_DP');
end

%% Surf plots
figure(1);
clf
ss_grid.plotGridProbability(grid_probability);
colorbar
colormap parula;
caxis([0 1]);
view(0, 90)
axis equal
axis([-1 1 -1 1]);
box on
xlabel('x')
ylabel('y')
zlabel('Safety probability');
set(gca,'FontSize',fontSize);


%% Contour plots
[X,Y] = ss_grid.getMeshGrids();
x = X(1,:);
x_ext = [x(1) - (x(2)-x(1)), x, x(end) + (x(2)-x(1))];
y = Y(:,1);
y_ext = [y(1) - (y(2)-y(1)); y; y(end) + (y(2)-y(1))];
grid_probability_mat = reshape(grid_probability, ss_grid.n_points);
grid_probability_mat_ext = zeros(size(grid_probability_mat,1)+2,size(grid_probability_mat,2)+2);
grid_probability_mat_ext(2:end-1,2:end-1)=grid_probability_mat;

figure(2);
clf
hold on
color_string = ['b','g','y'];
for i=1:2:length(alpha_vec)
    plot(underapproximate_stochastic_reach_avoid_polytope_ccc(i),'color',color_string(i),'alpha',1);
end
contour(x_ext, y_ext, grid_probability_mat_ext, alpha_vec([1,3]),'LineWidth',3);
colorbar
colormap parula;
caxis([0.5 1]);
axis equal
box on
xlabel('x')
ylabel('y')
set(gca,'FontSize',fontSize);
axis([-1-(x(2)-x(1)) 1+(x(2)-x(1)) -1-(y(2)-y(1)) 1+(y(2)-y(1))]);

figure(3)
clf
hold on
plot(underapproximate_stochastic_reach_avoid_polytope_ccc(2),'color',color_string(2),'alpha',1);
contour(x_ext, y_ext, grid_probability_mat_ext, [alpha_vec(2) alpha_vec(2)],'LineWidth',3);
colorbar
colormap parula;
caxis([0.5 1]);
axis equal
box on
xlabel('x')
ylabel('y')
set(gca,'FontSize',fontSize);
axis([-1-(x(2)-x(1)) 1+(x(2)-x(1)) -1-(y(2)-y(1)) 1+(y(2)-y(1))]);
% TODO: Create the interpolation code
% TODO: Compare the interpolated set with underapproximative set and the
% true set