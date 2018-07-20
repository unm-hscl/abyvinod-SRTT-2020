clear
clc
close all

load('CI_example.mat');

fontSize=40;

%% Open-loop underapproximation
figure(2);
clf
hold on
plot(target_tube(1).slice(n_zeroed+1:n_intg,zeros(n_intg-n_zeroed,1)),'color','y');
plot(target_tube(time_horizon + 1).slice(n_zeroed+1:n_intg,zeros(n_intg-n_zeroed,1)),'color','k','alpha',1);
color_string = ['g','g','b'];
for i=1:2:length(alpha_vec)
    plot(underapproximate_stochastic_reach_avoid_polytope_ccc(i).slice(n_zeroed+1:n_intg,zeros(n_intg-n_zeroed,1)),'color',color_string(i),'alpha',0.8);
end
leg=legend('Safe set','Target set','Open loop underapprox. (0.6)','Open loop underapprox. (0.9)');
set(leg,'Location','EastOutside');
axis equal
box on
xlabel('$x_1$','interpreter','latex')
ylabel('$x_2$','interpreter','latex')
set(gca,'FontSize',fontSize);
axis(1.1*[-10 10 -10 10]);
% savefig(gcf,'MATLAB_figs/CI_example_Figure1a.fig','compact');
% saveas(gcf,'MATLAB_figs/CI_example_Figure1a.png');

%% Interpolation comparison
figure(3)
clf
hold on
plot(target_tube(1).slice(n_zeroed+1:n_intg,zeros(n_intg-n_zeroed,1)),'color','y');
plot(target_tube(time_horizon + 1).slice(n_zeroed+1:n_intg,zeros(n_intg-n_zeroed,1)),'color','k','alpha',1);
plot(underapproximate_stochastic_reach_avoid_polytope_ccc(2).slice(n_zeroed+1:n_intg,zeros(n_intg-n_zeroed,1)),'color','w','alpha',1);
plot(interp_set.slice(n_zeroed+1:n_intg,zeros(n_intg-n_zeroed,1)),'color','b','alpha',0.5);
axis equal
box on
xlabel('$x_1$','interpreter','latex')
ylabel('$x_2$','interpreter','latex')
set(gca,'FontSize',fontSize);
axis(1.1*[-10 10 -10 10]);
leg=legend('Safe set','Target set','Open loop underapprox. (0.85)','Underapprox. interpolation (0.85)');
set(leg,'Location','EastOutside');
% savefig(gcf,'MATLAB_figs/DI_example_Figure2.fig','compact');
% saveas(gcf,'MATLAB_figs/DI_example_Figure2.png');

elapsed_time_polytope_ccc
elapsed_time_interp