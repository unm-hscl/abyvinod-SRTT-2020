clear
clc

dropboxpath='D:/Dropbox';
fontSize=20;
if exist(dropboxpath,'file') == 0
    warning('Switching the file name to Linux Dropbox.');
    dropboxpath='/datafiles/Dropbox';
    fontSize=40;
    if exist(dropboxpath,'file') == 0
        warning('Storing the mat files here.');
        dropboxpath = '.';
        fontSize=20;
    end    
end
save_mat_file_path = strcat(dropboxpath,'/MatFiles/2018TAC_Verification/',...
    'DI_example_',datestr(now,'YYYYmmDD_HHMMSS'),'.mat');

sampling_time = 0.1;                           % sampling time
time_horizon = 10;
alpha_vec = [0.6 0.85 0.9];
no_of_direction_vectors_ccc = 32;

%% System definition
n_intg = 2;
umax = 1;
xmax = [1,1];
dist_cov = 0.01;
sys = getChainOfIntegLtiSystem(n_intg,...
    sampling_time,...
    Polyhedron('lb',-umax,'ub',umax),...
    RandomVector('Gaussian', zeros(n_intg,1), dist_cov * eye(n_intg)));

%% Setup the target tube
% safe set definition
safe_set = Polyhedron('lb', -xmax, 'ub', xmax);
% target tube definition
target_tube = TargetTube('viability', safe_set, time_horizon);

% 2017 LCSS
% target_set = Polyhedron('lb', -[0.5,0.5], 'ub', [0.5,0.5]);
% safe_set = Polyhedron('lb', -xmax, 'ub', xmax);
% target_tube = TargetTube('reach-avoid', safe_set, target_set, time_horizon);
    

timer_DP=tic;
[prob_x, grid_x] = getDynProgSolForTargetTube2D(sys, 0.05, 0.05, target_tube);
elapsed_time_DP_recursion = toc(timer_DP);
x=grid_x(1:sqrt(length(grid_x)),2);
%% Computation of an underapproximative stochastic reach-avoid set
theta_vector_ccc = linspace(0, 2*pi, no_of_direction_vectors_ccc+1);
theta_vector_ccc = theta_vector_ccc(1:end-1);
set_of_direction_vectors_ccc = [cos(theta_vector_ccc); 
                                sin(theta_vector_ccc)];

i=1;
elapsed_time_polytope_ccc = zeros(length(alpha_vec),1);
underapproximate_stochastic_reach_avoid_polytope_ccc =...
    repmat(Polyhedron(),length(alpha_vec),1);
for prob_thresh_of_interest = alpha_vec
    fprintf('Chance constrained approach for alpha=%1.2f\n',...
        prob_thresh_of_interest);
    timer_polytope_ccc = tic;
    underapproximate_stochastic_reach_avoid_polytope_ccc(i) =...
        getUnderapproxStochReachAvoidSet(...
            sys, ...
            target_tube, ...
            [], ...
            prob_thresh_of_interest, ...
            set_of_direction_vectors_ccc,...
            'ccc');
    elapsed_time_polytope_ccc(i) = toc(timer_polytope_ccc);
    i = i+1;
end

x_ext = [x(1) - (x(2)-x(1)), x', x(end) + (x(2)-x(1))]';
grid_probability_mat = reshape(prob_x, length(x),[]);
grid_probability_mat_ext =...
    zeros(size(grid_probability_mat,1)+2,size(grid_probability_mat,2)+2);
grid_probability_mat_ext(2:end-1,2:end-1)=grid_probability_mat;

%% Open-loop underapproximation
figure(2);
clf
hold on
color_string = ['b','g','y'];
for i=1:2:length(alpha_vec)
    plot(underapproximate_stochastic_reach_avoid_polytope_ccc(i),...
        'color',color_string(i),'alpha',1);
end
contour(x_ext, x_ext, grid_probability_mat_ext, alpha_vec([1,3]),'LineWidth',3);
colorbar
colormap parula;
caxis([0.5 1]);
box on
xlabel('x')
ylabel('y')
set(gca,'FontSize',fontSize);
axis([x_ext(1) x_ext(end) x_ext(1) x_ext(end)]);
axis equal

%% OL interpolation
timer_interp = tic;
interp_set = interpStochReachAvoidSet(...
    alpha_vec(2),...
    underapproximate_stochastic_reach_avoid_polytope_ccc(3),...        
    alpha_vec(3),...
    underapproximate_stochastic_reach_avoid_polytope_ccc(1),...        
    alpha_vec(1));
elapsed_time_interp = toc(timer_interp);

%% DP interpolation
timer_DP_set = tic;
[C_DP]=contourc(x_ext, x_ext, grid_probability_mat_ext, alpha_vec([1,3]));
elapsed_time_DP_set = toc(timer_DP_set);
elapsed_time_DP_total = elapsed_time_DP_recursion + elapsed_time_DP_set;
vertices_DP_below = max(-1,min(1,C_DP(:,2:C_DP(2,1)+1)));
vertices_DP_above = max(-1,min(1,C_DP(:,C_DP(2,1)+3:end)));
poly_DP_above = Polyhedron('V',vertices_DP_above');
poly_DP_below = Polyhedron('V',vertices_DP_below');
timer_interp_DP = tic;
interp_set_DP = interpStochReachAvoidSet(...
    alpha_vec(2),...
    poly_DP_above,...        
    alpha_vec(3),...
    poly_DP_below,...        
    alpha_vec(1));
elapsed_time_interp_DP = toc(timer_interp_DP);

%% Save all the data
save(save_mat_file_path);
    
%% Interpolation comparison
figure(3)
clf
hold on
C_DP_middle = contourc(x_ext, x_ext, grid_probability_mat_ext, [alpha_vec(2) alpha_vec(2)]);
poly_DP_middle = Polyhedron('V',max(-1,min(1,C_DP_middle(:,2:end)))');
plot(poly_DP_middle,'color','r','alpha',0.8);
plot(interp_set_DP,'color','m','alpha',0.8);
plot(underapproximate_stochastic_reach_avoid_polytope_ccc(2),'color',color_string(2),'alpha',1);
plot(interp_set,'color','b','alpha',0.8);
axis equal
box on
xlabel('x')
ylabel('y')
set(gca,'FontSize',fontSize);
axis([x_ext(1) x_ext(end) x_ext(1) x_ext(end)]);

leg=legend('Dyn. prog.','Interpolation','Open-loop','Interpolation');
set(leg,'Location','EastOutside');
