clear
clc
close all

dropboxpath='D:/Dropbox';
if exist(dropboxpath,'file') == 0
    warning('Switching the file name to Linux Dropbox.');
    dropboxpath='/datafiles/Dropbox';
    if exist(dropboxpath,'file') == 0
        warning('Searching for the mat files here.');
        dropboxpath = '.';
    end    
end
date_str_mat = '20180719_1024XX';
load(strcat(dropboxpath,strcat('/MatFiles/2018TAC_Verification/CI_example_',date_str_mat,'.mat')));
fontSize=20;
savefigures = 0;

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
% leg=legend('Safe set','Target set','Open loop underapprox. (0.6)','Open loop underapprox. (0.9)');
% set(leg,'Location','EastOutside');
colorbar
colormap winter;
caxis([0 1]);
axis equal
box on
xlabel('$x_1$','interpreter','latex')
ylabel('$x_2$','interpreter','latex')
set(gca,'FontSize',fontSize);
axis(1.1*[-10 10 -10 10]);
if savefigures
    savefig(gcf,'MATLAB_figs/CI_example_FigureCI_a.fig','compact');
    saveas(gcf,'MATLAB_figs/CI_example_FigureCI_a.png');
end

%% Interpolation comparison
figure(3)
clf
hold on
plot(target_tube(1).slice(n_zeroed+1:n_intg,zeros(n_intg-n_zeroed,1)),'color','y');
plot(target_tube(time_horizon + 1).slice(n_zeroed+1:n_intg,zeros(n_intg-n_zeroed,1)),'color','k','alpha',1);
plot(underapproximate_stochastic_reach_avoid_polytope_ccc(2).slice(n_zeroed+1:n_intg,zeros(n_intg-n_zeroed,1)),'color','g','alpha',1);
plot(interp_set.slice(n_zeroed+1:n_intg,zeros(n_intg-n_zeroed,1)),'color','b','alpha',0.5);
axis equal
box on
xlabel('$x_1$','interpreter','latex')
ylabel('$x_2$','interpreter','latex')
set(gca,'FontSize',fontSize);
axis(1.1*[-10 10 -10 10]);
leg=legend('Safe set','Target set','Open loop underapprox. (0.85)','Underapprox. interpolation (0.85)');
set(leg,'Location','EastOutside');
if savefigures
    savefig(gcf,'MATLAB_figs/CI_example_FigureCI_b.fig','compact');
    saveas(gcf,'MATLAB_figs/CI_example_FigureCI_b.png');
end

disp('Time to compute the sets');
disp(elapsed_time_polytope_ccc)
disp('Time to compute interpolation');
disp(elapsed_time_interp);
disp('Difference between volumes of the 2D slices');
disp(volume(underapproximate_stochastic_reach_avoid_polytope_ccc(2).slice(n_zeroed+1:n_intg,zeros(n_intg-n_zeroed,1)))-volume(interp_set.slice(n_zeroed+1:n_intg,zeros(n_intg-n_zeroed,1))))