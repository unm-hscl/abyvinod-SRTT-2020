function interp_set = interpViaMinkSum(prob_thresh_of_interest,...
        polytope_above, prob_thresh_above, polytope_below, prob_thresh_below)
% Interpolate polytopic representations of two level sets for an 
% (underapproximative) polytopic representation of a desired level set
% ============================================================================
%
% Implements Theorem 12/Corollary 13.
%
% ============================================================================
%
% 
% Inputs:
% -------
%   prob_thresh_of_interest - Desired level set
%   polytope_above          - Polytopic representation of the higher-valued
%                               level set         
%   prob_thresh_above       - Higher valued level
%   polytope_below          - Polytopic representation of the lower-valued
%                               level set         
%   prob_thresh_below       - Lower valued level
%
% Outputs:
% --------
%   interp_set - Underapproximative polytopic representation of interpolated
%                   level set
%
% Notes:
% ------
%
% * NOT ACTIVELY TESTED: Not
% * MATLAB DEPENDENCY: None
% * EXTERNAL DEPENDENCY: MPT3
%
        
    % Compute the theta for the convex combination
    cvx_comb_theta = (log(prob_thresh_above)-log(prob_thresh_of_interest))/...
        (log(prob_thresh_above)-log(prob_thresh_below));
    
    % Corollary 13 implementation
    interp_set = cvx_comb_theta * polytope_below + (1 - cvx_comb_theta) * ...
        polytope_above;
end