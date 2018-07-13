clear
% dropboxpath='D:/Dropbox';
% fontSize=20;
dropboxpath='/datafiles/Dropbox';
fontSize=40;
% Run CWH_example with Figure3=1
load(strcat(dropboxpath,'/MatFiles/2018TAC_Verification/Figure3_8.mat'),...
    'safe_set','target_set','xmax_ccc','xmax_ft','slice_at_vx_vy',...
    'elapsed_time_polytope_genzps','elapsed_time_polytope_ccc',...
    'underapproximate_stochastic_reach_avoid_polytope_2D_ccc',...
    'underapproximate_stochastic_reach_avoid_polytope_2D_ft');
load(strcat(dropboxpath,'/MatFiles/2018TAC_Verification/cwh_save.mat'),...
    'DsetTemp');
fprintf('CCC: %1.2f\n',elapsed_time_polytope_ccc)
fprintf('FT: %1.2f\n',elapsed_time_polytope_genzps)

figure(100);
clf
hold on;
plot(safe_set.slice([3,4], slice_at_vx_vy), 'color', 'y');
plot(target_set.slice([3,4], slice_at_vx_vy), 'color', 'k');
plot(DsetTemp.slice([3,4], slice_at_vx_vy), 'color','c','alpha',1);
plot(underapproximate_stochastic_reach_avoid_polytope_2D_ccc,...
     'color','m','alpha',0.9);
plot(underapproximate_stochastic_reach_avoid_polytope_2D_ft,...
     'color','b','alpha',0.8);
scatter(xmax_ccc(1), xmax_ccc(2), 100,'ws','filled','MarkerEdgeColor','k')
scatter(xmax_ft(1), xmax_ft(2), 100,'gs','filled','MarkerEdgeColor','k')
leg=legend({'Safe set',...
        'Target set',...
        'Lagrangian',...
        'Chance constraint',...
        'Fourier transform',...
        '$\bar{x}_\mathrm{max}$ (Chance const.)',...
        '$\bar{x}_\mathrm{max}$ (Fourier tran.)'});
set(leg,'Location','EastOutside','interpreter','latex');
xlabel('x')
ylabel('y')
axis equal;
box on;
grid on;
axis equal
set(gca,'FontSize',fontSize)
axis([-2.5 2.5 -2 0]);

% savefig(gcf,'MATLAB_figs/CWH_example_Figure3a.fig','compact');
% saveas(gcf,'MATLAB_figs/CWH_example_Figure3a.png');