figure(100);
clf
hold on;
plot(safe_set.slice([3,4], slice_at_vx_vy), 'color', 'y');
plot(target_set.slice([3,4], slice_at_vx_vy), 'color', 'k');

scatter(xmax_ccc(1), xmax_ccc(2), 100,'gs','filled')
plot(underapproximate_stochastic_reach_avoid_polytope_2D_ccc,...
     'color','m','alpha',1);
plot(underapproximate_stochastic_reach_avoid_polytope_2D_ft,...
     'color','b','alpha',0.8);
leg=legend({'Safe set',...
        'Target set',...
        '$\bar{x}_\mathrm{max}$',...
        'Chance-constraint',...
        'Fourier transform'});
set(leg,'Location','South','interpreter','latex');
xlabel('x')
ylabel('y')
axis equal;
box on;
grid on;
axis equal
set(gca,'FontSize',20)
if figure3
    axis([-2.5 2.5 -2 0]);
else
    axis([-1 -0.8 -1.2 -0.9])
    set(leg,'Orientation','Vertical');
end
if savefig_true
    savefig(gcf,strcat(['MATLAB_figs/CWH_example_',figsave_str,'_initial_vel.fig']),'compact')
    saveas(gcf,strcat(['MATLAB_figs/CWH_example_',figsave_str,'_initial_vel.png']))
end
% load('D:/Dropbox/MatFiles/2018HSCC/Lesser_CCC_original.mat','x01','x02','Prob','timeSpent')
% caxis([0 1])
% colormap_matrix = colormap('copper');
% color_step_size = 1/(size(colormap_matrix,1)-1);
% colorpolytope = colormap_matrix(round((0.8-.3)/color_step_size)+1,:);
% [~,donotplot_handle]=contourf(x01,x02,Prob-.3,[.8-.3 .8-.3]);
% set(get(get(donotplot_handle,'Annotation'),'LegendInformation'),'IconDisplayStyle','off'); % Exclude line from legend
% plot(Polyhedron('lb',-ones(2,1),'ub',ones(2,1))*0.0001+[0;0.05], 'alpha',1,'color', colorpolytope,'LineStyle','-','LineWidth',0.001);
% plot(underapproximate_stochastic_reach_avoid_polytope_2D_ccc,...
% 'color','m','alpha',0.5);
% title(sprintf('Open-loop underapproximative\nstochastic reach-avoid set'));