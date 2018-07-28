function [v_interp] = equisample(angles, verts)
    % Expects vertices arranged columnwise
    % Expects direction_vectors arranged columnwise

%     % Create angles
%     dir_vecs_centered = dir_vecs - mean(dir_vecs,2);
%     angles = atan2(dir_vecs_centered(2,:), dir_vecs_centered(1,:));
    
    % Angles made by the vertices of the below polytope at the origin
    center_pt = mean(verts,2);
    v_angles = atan2(verts(2,:)-center_pt(2),verts(1,:)-center_pt(1));
    
    % Sort the angles
    [v_ang_sorted,sort_idx] = sort(v_angles);
    
    v_interp = zeros(2, length(angles));
    
    % MATLAB iterator requires options to be column vectors
    for angle_indx = 1:length(angles)
        % Iteration angle
        angle_iter = angles(angle_indx);
        % Find the closest angle
        [err,v_sorted_idx_A]  = min(abs(angle_iter - v_ang_sorted));
        % Translate to the original indexing (unsorted)
        v_idx_A = sort_idx(v_sorted_idx_A);
        
        if abs(err)<eps
            % Found an exact match for the vertex
            pointPq = verts(:,v_idx_A);
        else                    
            % Find the bounding angle
            if v_sorted_idx_A + 1 <=length(v_ang_sorted)...
                    && v_ang_sorted(v_sorted_idx_A) <= angle_iter...
                    &&  angle_iter <= v_ang_sorted(v_sorted_idx_A+1) 
                % We found a lower bound and the next one bounds
                v_idx_B = sort_idx(v_sorted_idx_A+1);
            elseif v_sorted_idx_A ==length(v_ang_sorted)...
                    && v_ang_sorted(end) <= angle_iter &&  angle_iter <= pi
                % We found a lower bound but it is the largest angle so wrap it
                % around
                v_idx_B = sort_idx(1);
            elseif v_sorted_idx_A >= 2 ...
                    && v_ang_sorted(v_sorted_idx_A-1) <= angle_iter...
                    &&  angle_iter <= v_ang_sorted(v_sorted_idx_A) 
                % We found an upper bound so the previous one bounds
                v_idx_B = sort_idx(v_sorted_idx_A-1);
            elseif v_sorted_idx_A == 1 ...
                    && -pi <= angle_iter &&  angle_iter <= v_ang_sorted(1) 
                % We found an upper bound but it is the first one, so wrap it
                % around
                v_idx_B = sort_idx(length(v_ang_sorted));
            else
                disp('No idea why we are here');
                keyboard
            end

            % Get the points
            pointA = verts(:,v_idx_A);
            pointB = verts(:,v_idx_B);

            %% Next we solve for the intersection point between (rcos(),rsin()) and 
            %% lines joining (a1,a2) and (b1,b2)
            % x = r*cos(angle_iter) + center_pt(1)
            % y = (a2 - b2)/(a1 - b1) * x + (a1*b2 - a2*b1)/(a1 - b1) = r*sin(angle_iter) + center_pt(2)
            P = [1                                               -cos(angle_iter);
                 (pointA(2) - pointB(2))/(pointA(1) - pointB(1)) -sin(angle_iter)];
            q = [0;
                -(pointA(1)*pointB(2) - pointA(2)*pointB(1))/(pointA(1) - pointB(1))] ... 
                + center_pt;

            if abs(pointA(1) - pointB(1))<eps && abs(cos(angle_iter))>eps
                % the line joining the points are parallel to y-axis and the ray is 
                % away from y-axis | q(1) and P(1,1) will blow up to Inf
                disp('pointA.x=pointB.x => intersect.x=pointA.x');
                xr = [pointA(1);
                      (pointA(1)-center_pt(1))/cos(angle_iter)];
            elseif abs(pointA(1) - pointB(1))<eps && abs(cos(angle_iter))<=eps
                % the line joining the points are parallel to y-axis and the ray is 
                % along the y-axis | q(1) and P(1,1) will blow up to Inf
                disp('pointA and pointB are along y-axis => choose the furthest point');
                if abs(pointA(2)-center_pt(2)) >= abs(pointB(2)-center_pt(2))
                    xr = [0;
                          (pointA(2)-center_pt(2))];
                else
                    xr = [0;
                          (pointB(2)-center_pt(2))];
                end
            else
                disp('Normal intersection');
%                 xr = P\q;
                xr = [0;0];
            end
            pointPq = [xr(2)* cos(angle_iter); 
                       xr(2)* sin(angle_iter)] + center_pt;
        end
        v_interp(:,angle_indx) = pointPq;
    end
end