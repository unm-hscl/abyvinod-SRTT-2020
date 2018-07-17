function [prob_x, grid_x] = getDynProgSolForTargetTube2D(sys, x_inc, u_inc, target_tube)
% SReachTools/stochasticReachAvoid/getDynProgSolForTargetTube Get dynamic 
% programming grid probability for reachability of target tube
% ============================================================================
%
% The function computes the probability of staying in a target tube defined
% on a particular state stace grid. The dynamic programming recursion can be
% found in 
%   
% S. Summers and J. Lygeros, "Verification of discrete time stochastic hybrid 
% systems: A stochastic reach-avoid decision problem," Automatica, vol. 46,
% no. 12, pp. 1951--1961, 2010.
%
% The problem of examining the reachability of a target tube can be found in
% a work that we intend to publish soon :D TODO
%
% Usage: See example doubleIntegratorDynmaicProgramming.m
%
% ============================================================================
%
% grid_prob = getDynProgSolForTargetTube(sys, ...
%     state_grid, input_grid, target_tube, varargin)
% 
% Inputs:
% -------
%   sys         - LtiSystem object
%   state_grid  - SpaceGrid object
%   input_grid  - InputGrid object
%   target_tube
%               - Target tube of length N+1 where N is the time_horizon. It should have
%                   polyhedrons T_0, T_1,...,T_N.
%
% Outputs:
% --------
%   grid_prob   - Nx1 Array of probability values, where N is equivalent
%                 to size(state_grid, 1)
%
% Notes:
% ------
% * WARNING: Dynamic programming suffers from the curse of dimensionality! As
%   such, this code will effective and with reasonable speed compute dynamic
%   programming solutions for 2-dimensional systems with Gaussian 
%   disturbances. However, for 3-dimensional systems or larger the required
%   computation time, and memory, will exponentially grow to the point that 
%   the simulation will take longer than it took for me to get my PhD.
% * Currently this back propagation, and subsequently the entire dynamic 
%   programming recursion, only works for Gaussian disturbances.
% 
% ============================================================================
% 
%   This function is part of the Stochastic Reachability Toolbox.
%   License for the use of this function is given in
%        https://github.com/unm-hscl/SReachTools/blob/master/LICENSE
%
    % check inputs
    validateattributes(sys, {'LtiSystem'}, {'nonempty'})
    validateattributes(target_tube, {'TargetTube'}, {'nonempty'});
    
    validateattributes(sys.dist, {'RandomVector', 'StochasticDisturbance'}, ...
        {'nonempty'});
    if ~strcmpi(sys.dist.type, 'Gaussian')
        throwAsCaller(SrtInvalidArgsError('Handles only Gaussian disturbances'));
    end
    
    % n_targets is time_horizon + 1
    n_targets = length(target_tube);

    % Computation of corners
    vertices_target_sets = [];
    for itt = 1:n_targets
        vertices_target_sets = [vertices_target_sets;
            target_tube(itt).V];
    end
    xmax = max(vertices_target_sets);
    xmin = min(vertices_target_sets);
    
    if sys.state_dim == 1
        x1vec = xmin(1)-x_inc/2:x_inc:xmax(1)+x_inc/2;
%         x1vec = xmin(1):x_inc:xmax(1);
        grid_x = allcomb(x1vec);
    elseif sys.state_dim == 2
        x1vec = xmin(1)-x_inc/2:x_inc:xmax(1)+x_inc/2;
        x2vec = xmin(2)-x_inc/2:x_inc:xmax(2)+x_inc/2;
%         x1vec = xmin(1):x_inc:xmax(1);
%         x2vec = xmin(2):x_inc:xmax(2);
        grid_x = allcomb(x1vec,x2vec);
    elseif sys.state_dim == 3
        x1vec = xmin(1)-x_inc/2:x_inc:xmax(1)+x_inc/2;
        x2vec = xmin(2)-x_inc/2:x_inc:xmax(2)+x_inc/2;
        x3vec = xmin(3)-x_inc/2:x_inc:xmax(3)+x_inc/2;
%         x1vec = xmin(1):x_inc:xmax(1);
%         x2vec = xmin(2):x_inc:xmax(2);
%         x3vec = xmin(3):x_inc:xmax(3);
        grid_x = allcomb(x1vec,x2vec,x3vec);        
    else
        throwAsCaller(SrtInvalidArgsError('System can have at most 3 states'));
    end
    
    n_grid_x = length(grid_x);
    
    % Input gridding
    umax = max(sys.input_space.V);
    umin = min(sys.input_space.V);
    if sys.input_dim == 1
        grid_u = allcomb(umin(1):u_inc:umax(1));
    elseif sys.input_dim == 2
        grid_u = allcomb(umin(1):u_inc:umax(1),umin(2):u_inc:umax(2));
    elseif sys.input_dim == 3
        grid_u = allcomb(umin(1):u_inc:umax(1),umin(2):u_inc:umax(2),umin(3):u_inc:umax(3));
    else
        throwAsCaller(SrtInvalidArgsError('System can have at most 3 inputs'));
    end
    
    
    fprintf('Set optimal value function at t=%d\n',n_targets-1);    
    terminal_indicator_x = target_tube(n_targets).contains(grid_x');
    prob_x = terminal_indicator_x;
    
    transition_prob = getTransProb(sys, grid_x, grid_u);
    for itt = n_targets - 1:-1:1
        fprintf('Compute optimal value function at t=%d\n', itt - 1);
        old_prob_x = prob_x;
        current_indicator_x = target_tube(itt).contains(grid_x');
        prob_x = zeros(1,n_grid_x);
        for ix = find(current_indicator_x==1)
            prob_x(ix) = max(old_prob_x*transition_prob{ix}');
        end        
    end
end

function transition_prob = getTransProb(sys, grid_x, grid_u)

    % Define transition_prob as a cell array
    transition_prob = cell(length(grid_x),1);
    
    % \int 1_Box(0,0.5) dx = 1 => \sum\sum delta_x = 1 => delta_x = 1/N^2
    % where N is the number of points of grid_x within the unit box.
    xmax = max(grid_x);
    xmin = min(grid_x);
    if min(xmax)<0.5 || max(xmin)>-0.5
        throwAsCaller(SrtInvalidArgsError('Delta_x computation made invalid since unit box not contained in the grid.'));
    end
    box_corner = 0.5;
    N_in_box = sum(Polyhedron('lb',-box_corner*ones(sys.state_dim,1),'ub',box_corner*ones(sys.state_dim,1)).contains(grid_x'));
    delta_x = (2*box_corner)^sys.state_dim/N_in_box;
    
    n_grid_x = length(grid_x);
    dist_cov = sys.state_mat * sys.dist.parameters.covariance * sys.state_mat';            
    fprintf('Compute transition probability...000%%');
    
    % For printing stuff --- Create fixed markers in the index space
    print_marker = linspace(1,n_grid_x,100+1);
    print_marker(end) = print_marker(end)-1;
    print_marker_indx = 1;
    print_marker_val = (print_marker(2)-print_marker(1))/n_grid_x*100;
    
    for ix = 1:n_grid_x
        transition_prob{ix} = zeros(length(grid_u), length(grid_x));
        for iu = 1:length(grid_u)
            dist_mean = sys.state_mat * grid_x(ix,:)' + sys.input_mat * grid_u(iu,:) + sys.dist_mat * sys.dist.parameters.mean;
            transition_prob{ix}(iu,:) = mvnpdf(grid_x, dist_mean', dist_cov)' * delta_x;
        end
        if ix > print_marker(print_marker_indx)
            val = (print_marker_indx-1) * print_marker_val;
            fprintf('\b\b\b\b%3d%%', round(val))
            print_marker_indx = print_marker_indx + 1;
        end
    end
    fprintf('\n');
end
