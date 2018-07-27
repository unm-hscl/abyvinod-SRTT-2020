function [vertices_interp] = equisample(direction_vectors, vertices)
    % Create angles
    angles = atan2(direction_vectors(:,2), direction_vectors(:,1));
    % Angles made by the vertices of the below polytope at the origin
    vertices_angles = atan2(vertices(:,2),vertices(:,1));
    [vertices_angles_sorted,sort_idx] = sort(vertices_angles);
    vertices_interp = [];
    for angle_iter = angles'
        [~,vertices_sorted_idx_A]  = min(abs(angle_iter - vertices_angles_sorted));
        vertices_idx_A = sort_idx(vertices_sorted_idx_A);
        if vertices_sorted_idx_A + 1 <=length(vertices_angles_sorted) && vertices_angles_sorted(vertices_sorted_idx_A,:) <= angle_iter &&  angle_iter <= vertices_angles_sorted(vertices_sorted_idx_A+1,:) 
            vertices_idx_B = sort_idx(vertices_sorted_idx_A+1);
        elseif vertices_sorted_idx_A ==length(vertices_angles_sorted) && vertices_angles_sorted(vertices_sorted_idx_A,:) <= angle_iter &&  angle_iter <= pi
            vertices_idx_B = sort_idx(1);
        elseif vertices_sorted_idx_A >= 2 && vertices_angles_sorted(vertices_sorted_idx_A-1,:) <= angle_iter &&  angle_iter <= vertices_angles_sorted(vertices_sorted_idx_A,:) 
            vertices_idx_B = sort_idx(vertices_sorted_idx_A-1);
        elseif vertices_sorted_idx_A == 1 && -pi <= angle_iter &&  angle_iter <= vertices_angles_sorted(vertices_sorted_idx_A,:) 
            vertices_idx_B = sort_idx(length(vertices_angles_sorted));
        else
            disp('No idea why we are here');
            keyboard
        end
        pointA = vertices(vertices_idx_A,:);
        pointB = vertices(vertices_idx_B,:);
        % Px=q
        P = [(pointA(2) - pointB(2))/(pointA(1) - pointB(1)) -sin(angle_iter);
             1                                               -cos(angle_iter)];
        q = [-(pointA(1)*pointB(2) - pointA(2)*pointB(1))/(pointA(1) - pointB(1)); 0];
        if abs(pointA(1) - pointB(1))<eps && abs(cos(angle_iter))>eps
            % q(1) and P(1,1) will blow up to Inf
            disp('pointA.x=pointB.x => intersect.x=pointA.x');
            xr = [pointA(1);
                  pointA(1)/cos(angle_iter)];
        elseif abs(pointA(1) - pointB(1))<eps && abs(cos(angle_iter))<=eps
            % q(1) and P(1,1) will blow up to Inf
            disp('pointA and pointB are along y-axis => choose the furthest point');
            [~, max_idx] = max(abs(pointA(2)),abs(pointB(2)));
            if max_idx == 1
                xr = [0;pointA(2)];
            else
                xr = [0;pointB(2)];
            end
        else
            xr = P\q;
        end
        pointPq=[xr(2)* cos(angle_iter); xr(2)* sin(angle_iter)];
        vertices_interp = [vertices_interp, pointPq];
    end
    poly_DP_below_interp = Polyhedron('V',vertices_interp');
end