
function [left_tangent_3d, right_tangent_3d,top_tangent] =compute_tangents(point3D,nextPoint3D, poly3D)
% point3D: 1x3 coordinates of the external point
% poly3D: struct with fields:
%   - vertices: Nx2 matrix of convex polygon base vertices
%   - z_min: minimum z-value (bottom)
%   - z_max: maximum z-value (top)
% Returns:
%   - left_tangent_3d, right_tangent_3d: 1x3 points on polygon edges

point2D = point3D(1:2);
polygon = poly3D.vertices;
D = nextPoint3D - point3D;
line_length = norm(D);
is_vertex = any(ismember(polygon, point2D, 'rows'));

if is_vertex
    idx = find(ismember(polygon, point2D, 'rows'));
    n = size(polygon, 1);

    left_tangent_2d = polygon(mod(idx - 2, n) + 1, :);
    right_tangent_2d = polygon(mod(idx, n) + 1, :);
else
    n = size(polygon, 1);
    left_tangent_2d = polygon(1, :);
    right_tangent_2d = polygon(1, :);

    for i = 2:n
        if is_below(point2D, left_tangent_2d, polygon(i, :))
            left_tangent_2d = polygon(i, :);
        end
    end
    for i = 2:n
        if is_above(point2D, right_tangent_2d, polygon(i, :))
            right_tangent_2d = polygon(i, :);
        end
    end
end
z_tangent = poly3D.height;
z_min = 0;


int_points = line_prism_intersection(point3D, D, polygon, z_min, z_tangent,line_length);

if ~isempty(int_points)
    int_point = int_points(1,:);
    int_point(3) = z_tangent+6;
    top_tangent = int_point;
else
    top_tangent = [NaN NaN NaN];
end
% Use the z of the original point, or use z_max for consistency with the top of the polygon

z_left = compute_z(point3D, nextPoint3D, left_tangent_2d(1,:));
z_right = compute_z(point3D, nextPoint3D, right_tangent_2d(1,:));
% Lift 2D tangents to 3D
left_tangent_3d = [left_tangent_2d(1,:), z_left];
right_tangent_3d = [right_tangent_2d(1,:),z_right];



    function z_T = compute_z(P1, P3, T_xy)
        % P1, P3: 1x3 vectors defining the line
        % T_xy: 1x2 point (x,y) on the line
        % Output: z_T - interpolated z value at T_xy

        dir2D = P3(1:2) - P1(1:2);  % direction in XY plane
        delta2D = T_xy - P1(1:2);   % vector from P1 to the tangent point

        % Use dot product projection to compute parameter t along the segment
        t = dot(delta2D, dir2D) / dot(dir2D, dir2D);

        % Clamp t to [0,1] for safety (optional)
        t = max(0, min(1, t));

        % Interpolate z
        z_T = P1(3) + t * (P3(3) - P1(3));
    end


% Define the 'is_below' function
    function result = is_below(p, p1, p2)
        % Check if point p is below the line segment p1-p2
        result = (p(2) - p1(2)) * (p2(1) - p1(1)) < (p(1) - p1(1)) * (p2(2) - p1(2));
    end

% Define the 'is_above' function
    function result = is_above(p, p1, p2)
        % Check if point p is above the line segment p1-p2
        result = (p(2) - p1(2)) * (p2(1) - p1(1)) > (p(1) - p1(1)) * (p2(2) - p1(2));
    end



end

