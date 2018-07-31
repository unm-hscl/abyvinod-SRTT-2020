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
date_str_mat = '20180726_140648';%'20180721_191802';
% load(strcat(dropboxpath,strcat('/MatFiles/2018TAC_Verification/DI_example_',date_str_mat,'.mat')));
load('test.mat')
savefigures = 0;
fontSize=20;

%% Open-loop underapproximation
figure(2);
clf
hold on
color_string = ['b','g','g'];
plot(safe_set,'color','y');
for i=1:2:length(alpha_vec)
    plot(underapproximate_stochastic_reach_avoid_polytope_ccc(i),'color',color_string(i),'alpha',1);
end
[c,h]=contour(x, x, grid_probability_mat, alpha_vec([1,3]),'LineWidth',3);
fprintf('Choose contours and set 0.9 label colors to white\n');
clabel(c,h,'manual','FontSize',fontSize,'color','m');
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
C_DP_middle = contourc(x, x, grid_probability_mat, [alpha_vec(2) alpha_vec(2)]);
poly_DP_middle = Polyhedron('V',max(-1,min(1,C_DP_middle(:,2:end)))');
% plot(poly_DP_middle,'color','k','alpha',1);
[c,h_contour]=contour(x, x, grid_probability_mat, alpha_vec([2,2]),'LineWidth',3);
% clabel(c,h_contour,'FontSize',2*fontSize/3);
h_interp_DP = plot(interp_set_DP,'color','g','alpha',0.5);
plot(interp_set_DP,'color','b','alpha',0.3);
h_OL = plot(underapproximate_stochastic_reach_avoid_polytope_ccc(2),'color','m','alpha',1);
h_interp_OL = plot(interp_set,'color','c','alpha',1);
fprintf('Choose contours and set 0.9 label colors to white\n');
clabel(c,h_contour,'manual','FontSize',fontSize,'color','k');
axis equal
box on
xlabel('$x_1$','interpreter','latex')
ylabel('$x_2$','interpreter','latex')
set(gca,'FontSize',fontSize);
axis(1.1*[-xmax(1) xmax(1) -xmax(2) xmax(2)]);
legend_cell = {'Safe set','Dynamic programming','Interpolation (Thm. 5)','Open loop underapprox.','Interpolation (Prop. 6d)'};
leg=legend([h_safe, h_contour, h_interp_DP, h_OL, h_interp_OL], legend_cell{:});
set(leg,'Location','EastOutside');
if savefigures
    savefig(gcf,'MATLAB_figs/DI_example_FigureDI_b.fig','compact');
    saveas(gcf,'MATLAB_figs/DI_example_FigureDI_b.png');
end

%% Compute times
disp('Computation for Algorithm 1');
disp(elapsed_time_polytope_ccc)
disp(elapsed_time_interp)
fprintf('Computation time DP: Total:%1.3f | Elapsed: %1.3f\n', elapsed_time_DP_total, elapsed_time_interp_DP);
disp('Difference between volumes of the 2D slices');
disp(volume(underapproximate_stochastic_reach_avoid_polytope_ccc(2))-volume(interp_set))
