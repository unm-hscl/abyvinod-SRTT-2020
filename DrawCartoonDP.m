close all
clc

savefigures = 1;
%% System definition
sampling_time = 0.1;
n_intg = 1;
umax = 1;
xmax = 1;
xinc = 0.01;
dist_cov = 0.005;
sys = getChainOfIntegLtiSystem(n_intg,...
    sampling_time,...
    Polyhedron('lb',-umax,'ub',umax),...
    RandomVector('Gaussian', zeros(n_intg,1), dist_cov * eye(n_intg)));

%% Setup the target tube
% safe set definition
safe_set = Polyhedron('lb', -xmax, 'ub', xmax);
% target tube definition
gamma_val = 0.6;
target_tube = TargetTube(gamma_val^0 * safe_set,...
                         gamma_val^1 * safe_set,...
                         gamma_val^2 * safe_set,...
                         gamma_val^3 * safe_set,...
                         gamma_val^4 * safe_set);

[prob_x, grid_x, mat_prob_x] = getDynProgSolForTargetTube2D(sys, xinc, 0.05, target_tube);

n_grid_x = length(grid_x);
%% Plotting
figure(1);
clf
hold on;
for itt = 1:length(target_tube)
    stem3(grid_x, (itt-1)*ones(n_grid_x,1), mat_prob_x(itt,:)','b');
    current_indicator = target_tube(itt).contains(grid_x');
    plot3(grid_x, (itt-1)*ones(n_grid_x,1), current_indicator,'r','LineWidth',2);
    if any(current_indicator==0)
        xborder = grid_x(cumsum(current_indicator) == 1);
    else
        xborder = -xmax;
    end
    plot3([-xmax xborder xborder -xborder -xborder xmax], (itt-1)*ones(1,6), [0 0 1 1 0 0],'r','LineWidth',2);
end
plot3(grid_x, zeros(n_grid_x, 1), 0.8*ones(n_grid_x,1),'k','LineWidth',2);
set_defn = mat_prob_x(1,:) >=0.8;
stem3(grid_x(set_defn)', zeros(nnz(set_defn),1), mat_prob_x(1,set_defn)','k','filled','MarkerSize',5);
stem3(grid_x(set_defn)', zeros(nnz(set_defn),1), zeros(nnz(set_defn),1),'k','filled','MarkerSize',5);
box on
% axis equal
axis([-1 1 0 length(target_tube)-1 0 1])
set(gca,'ytick',0:length(target_tube)-1);
grid on
view([67 25]);
xlabel('$\overline{x}$','interpreter','latex');
ylabel('$k$','interpreter','latex');
% zlabel('$V_k^\ast(\overline{x})\ and\ 1_{\mathcal{T}_k}(\overline{x})$','interpreter','latex');
zlabel('$V_k^\ast(\overline{x})$','interpreter','latex');
if savefigures
    savefig(gcf,'MATLAB_figs/CartoonForDP.fig','compact');
    saveas(gcf,'MATLAB_figs/CartoonForDP.png');
end
