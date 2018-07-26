function [poly_array, grid_probability_mat] = getDynProgLevelSets(x, prob_x, interest_levels)
    grid_probability_mat = reshape(prob_x, length(x),[]);
    [C_DP]=contourc(x, x, grid_probability_mat, interest_levels);
    col_indx = 1;
    poly_array_vertices = cell(length(interest_levels),1);
    while col_indx <= length(C_DP)
        current_level = C_DP(1,col_indx);
        current_level_indx = find(abs(interest_levels - current_level)<eps);
        no_of_points = C_DP(2,col_indx);
        poly_array_vertices{current_level_indx} = [poly_array_vertices{current_level_indx},...
                                                   C_DP(:,col_indx+1:col_indx+no_of_points)];
        col_indx = col_indx + no_of_points + 1;        
    end
    poly_array = repmat(Polyhedron.emptySet(2),length(interest_levels),1);
    for poly_indx = 1:length(interest_levels)
        poly_array(poly_indx) = Polyhedron('V',poly_array_vertices{poly_indx}');        
    end
end