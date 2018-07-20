clear
clc
close all

dropboxpath='D:/Dropbox';
% dropboxpath='/datafiles/Dropbox';
load(strcat(dropboxpath,'/MatFiles/2018TAC_Verification/DI_example_20185420_130754.mat'));

fontSize=40;
savefigures = 0;

%% Surf plots
% figure(1);
% clf
% ss_grid.plotGridProbability(grid_probability);
% colorbar
% colormap parula;
% caxis([0 1]);
% view(0, 90)
% axis equal
% axis([-1 1 -1 1]);
% box on
% xlabel('$x_1$','interpreter','latex')
% ylabel('$x_2$','interpreter','latex')
% zlabel('Safety probability');
% set(gca,'FontSize',fontSize);

%% Contour plots
x_ext = [x(1) - (x(2)-x(1)), x', x(end) + (x(2)-x(1))]';
grid_probability_mat = reshape(prob_x, length(x),[]);
grid_probability_mat_ext = zeros(size(grid_probability_mat,1)+2,size(grid_probability_mat,2)+2);
grid_probability_mat_ext(2:end-1,2:end-1)=grid_probability_mat;

%% Open-loop underapproximation
figure(2);
clf
hold on
color_string = ['b','g','g'];
plot(safe_set,'color','y');
for i=1:2:length(alpha_vec)
    plot(underapproximate_stochastic_reach_avoid_polytope_ccc(i),'color',color_string(i),'alpha',1);
end
[c,h]=contour(x_ext, x_ext, grid_probability_mat_ext, alpha_vec([1,3]),'LineWidth',3);
fprintf('Choose contours and set 0.9 label colors to white\n');
clabel(c,h,'manual','FontSize',2*fontSize/3);
% clabel(c,h,'FontSize',2*fontSize/3);
colorbar
colormap winter;
caxis([0 1]);
axis equal
box on
xlabel('$x_1$','interpreter','latex')
ylabel('$x_2$','interpreter','latex')
set(gca,'FontSize',fontSize);
axis(1.1*[-xmax(1) xmax(1) -xmax(2) xmax(2)]);
if savefigures
    savefig(gcf,'MATLAB_figs/DI_example_FigureDI_a.fig','compact');
    saveas(gcf,'MATLAB_figs/DI_example_FigureDI_a.png');
end
  
%% Interpolation comparison
figure(3)
clf
hold on
h_safe = plot(safe_set,'color','y');
C_DP_middle = contourc(x_ext, x_ext, grid_probability_mat_ext, [alpha_vec(2) alpha_vec(2)]);
poly_DP_middle = Polyhedron('V',max(-1,min(1,C_DP_middle(:,2:end)))');
% plot(poly_DP_middle,'color','k','alpha',1);
[c,h_contour]=contour(x_ext, x_ext, grid_probability_mat_ext, alpha_vec([2,2]),'LineWidth',3);
% clabel(c,h_contour,'FontSize',2*fontSize/3);
h_interp_DP = plot(interp_set_DP,'color','g','alpha',0.5);
plot(interp_set_DP,'color','b','alpha',0.3);
h_OL = plot(underapproximate_stochastic_reach_avoid_polytope_ccc(2),'color','m','alpha',1);
h_interp_OL = plot(interp_set,'color','c','alpha',1);
fprintf('Choose contours and set 0.9 label colors to white\n');
clabel(c,h_contour,'manual','FontSize',2*fontSize/3);
axis equal
box on
xlabel('$x_1$','interpreter','latex')
ylabel('$x_2$','interpreter','latex')
set(gca,'FontSize',fontSize);
axis(1.1*[-xmax(1) xmax(1) -xmax(2) xmax(2)]);
legend_cell = {'Safe set','Dynamic programming','Underapprox. interpolation','Open loop underapprox.','Underapprox. interpolation'};
leg=legend([h_safe, h_contour, h_interp_DP, h_OL, h_interp_OL], legend_cell{:});
set(leg,'Location','EastOutside');
if savefigures
    savefig(gcf,'MATLAB_figs/DI_example_FigureDI_a.fig','compact');
    saveas(gcf,'MATLAB_figs/DI_example_FigureDI_a.png');
end


elapsed_time_polytope_ccc
elapsed_time_interp
elapsed_time_DP_total
elapsed_time_interp_DP
