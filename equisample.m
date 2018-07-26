function [vertices_interp] = equisample(angles, vertices)
    % Angles made by the vertices of the below polytope at the origin
    vertices_angles = atan2(vertices_DP_below(:,2),vertices_DP_below(:,1));
    vertices_DP_below_angles_interp = [];
    for angle_iter = angles'
        [~,vertices_idx_A]  = min(abs(angle_iter - vertices_angles));
        if vertices_idx_A + 1 <=length(vertices_angles) && vertices_angles(vertices_idx_A,:) <= angle_iter &&  angle_iter <= vertices_angles(vertices_idx_A+1,:) 
            vertices_idx_B = vertices_idx_A+1;
        elseif vertices_idx_A ==length(vertices_angles) && vertices_angles(vertices_idx_A,:) <= angle_iter &&  angle_iter <= vertices_angles(1,:) 
            vertices_idx_B = 1;
        elseif vertices_idx_A >= 2 && vertices_angles(vertices_idx_A-1,:) <= angle_iter &&  angle_iter <= vertices_angles(vertices_idx_A,:) 
            vertices_idx_B = vertices_idx_A-1;
        elseif vertices_idx_A == 1 && vertices_angles(end,:) <= angle_iter &&  angle_iter <= vertices_angles(vertices_idx_A,:) 
            vertices_idx_B = length(vertices_angles);
        else
            disp('No idea why we are here');
            keyboard
        end
        pointA = vertices_DP_below(vertices_idx_A,:);
        pointB = vertices_DP_below(vertices_idx_B,:);
        % Px=q
        P = [(pointA(2) - pointB(2))/(pointA(1) - pointB(1)) -sin(angle_iter);
             1                                               -cos(angle_iter)];
        q = [-(pointA(1)*pointB(2) - pointA(2)*pointB(1))/(pointA(1) - pointB(1)); 0];
        if abs(pointA(1) - pointB(1))<eps && abs(cos(angle_iter))>eps
            % q(1) and P(1,1) will blow up to Inf
            disp('pointA.x=pointB.x => intersect.x=pointA.x');
            xr = [pointA(1);
                  pointA(1)/cos(angle_iter)];
            keyboard
        elseif abs(pointA(1) - pointB(1))<eps && abs(cos(angle_iter))<=eps
            % q(1) and P(1,1) will blow up to Inf
            disp('pointA and pointB are along y-axis => choose the furthest point');
            xr = [0;max(pointA(2),pointB(2))];
            keyboard        
        else
            xr = P\q;
        end
        pointPq=[xr(2)* cos(angle_iter); xr(2)* sin(angle_iter)];
        keyboard
        vertices_DP_below_angles_interp = [vertices_DP_below_angles_interp, pointPq];
    end
    poly_DP_below_interp = Polyhedron('V',vertices_DP_below_angles_interp');
end