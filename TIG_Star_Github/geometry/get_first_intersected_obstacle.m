function [closest_obstacle, intersected_obstacles, obs_index, int_pnt] = get_first_intersected_obstacle(A, B, obstacles)
% Initialize outputs
int_pnt = [];
obs_index = -1;
closest_obstacle = [];
intersected_obstacles = {};
min_distance = inf;

% Line segment direction and parameterization
D = B - A; % Direction vector from A to B
line_length = norm(D);
if line_length < 1e-10
    return; % Line segment is degenerate
end
D_normalized = D / line_length; % Normalized direction

% Preallocate for efficiency
max_intersections = 10; % Adjust based on expected max intersections per prism
valid_points = zeros(max_intersections, 3);
num_valid = 0;

for i = 1:length(obstacles)
    prism = obstacles{i};
    polygon = prism.vertices; % 2D convex polygon vertices
    zmin = prism.z_range(1);
    zmax = prism.z_range(2);

    % Bounding box check
    min_x = min(polygon(:,1)); max_x = max(polygon(:,1));
    min_y = min(polygon(:,2)); max_y = max(polygon(:,2));
    if max(A(1), B(1)) < min_x || min(A(1), B(1)) > max_x || ...
            max(A(2), B(2)) < min_y || min(A(2), B(2)) > max_y || ...
            max(A(3), B(3)) < zmin || min(A(3), B(3)) > zmax
        continue; % Skip prism
    end

    % Compute intersection points with the prism
    intersection_points = line_prism_intersection(A, D_normalized, polygon, zmin, zmax, line_length);

    % Filter points to ensure they lie within the segment A to B
    num_valid = 0;
    for j = 1:size(intersection_points, 1)
        point = intersection_points(j, :);
        t = norm(point - A) / line_length; % Parameter t along segment
        if t >= 0 && t <= 1 && norm(point - B) > 1e-10 && norm(point - A) > 1e-10
            num_valid = num_valid + 1;
            if num_valid > size(valid_points, 1)
                valid_points = [valid_points; zeros(max_intersections, 3)]; %#ok<AGROW>
            end
            valid_points(num_valid, :) = point;
        end
    end

    % If there are valid intersections
    if num_valid > 0
        intersected_obstacles{end+1} = prism;
        % Find the closest intersection point incrementally
        for j = 1:num_valid
            dist = norm(valid_points(j, :) - A);
            if dist < min_distance
                closest_obstacle = prism;
                obs_index = i;
                min_distance = dist;
                int_pnt = valid_points(1:num_valid, :);
            end
        end
    end
end
end




