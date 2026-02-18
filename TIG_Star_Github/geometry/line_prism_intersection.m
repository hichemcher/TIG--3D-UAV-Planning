function points = line_prism_intersection(P0, D, vertices, z_min, z_max, line_length)
% P0: Starting point of the line segment (1x3)
% D: Direction vector (1x3, normalized or unnormalized)
% vertices: Polygon vertices (Nx2 matrix)
% z_min, z_max: Z-range of the prism
% line_length: Length of the line segment (scalar)

% Initialize output
max_points = size(vertices, 1) + 2; % Max possible intersections (sides + top/bottom)
points = zeros(max_points, 3);
num_points = 0;
epsilon = 1e-10;

% Ensure D is a row vector and normalized
D = D(:)'; % Convert to 1x3 if necessary
if line_length < epsilon
    return; % Degenerate line segment
end
D_normalized = D / norm(D); % Normalize direction

% --- Top and Bottom Faces ---
z_planes = [z_min, z_max];
for z_target = z_planes
    if abs(D_normalized(3)) > epsilon
        t = (z_target - P0(3)) / D_normalized(3);
        if t >= 0 && t <= line_length
            p = P0 + t * D_normalized;
            % Simplified convex polygon containment check
            if isPointInConvexPolygon(p(1:2), vertices)
                num_points = num_points + 1;
                points(num_points, :) = p;
            end
        end
    end
end

% --- Side Faces (Vectorized) ---
n = size(vertices, 1) - 1;
v1 = vertices(1:n, :);
v2 = vertices(2:n+1, :);
edges = v2 - v1;
normal_2D = [edges(:, 2), -edges(:, 1)];
normal_3D = [normal_2D, zeros(n, 1)];
plane_points = [v1, zeros(n, 1)];

% Compute intersection parameters
denom = dot(normal_3D, repmat(D_normalized, n, 1), 2);
valid = abs(denom) > epsilon;
t = dot(plane_points - repmat(P0, n, 1), normal_3D, 2) ./ denom;
valid = valid & (t >= 0) & (t <= line_length);

% Compute intersection points
p = repmat(P0, n, 1) + t .* repmat(D_normalized, n, 1);
vec = p(:, 1:2) - v1;
edge_lengths = sum(edges.^2, 2);
proj_len = sum(vec .* edges, 2) ./ edge_lengths;

% Check if points lie on edges and within z-range
for i = 1:n
    if valid(i) && proj_len(i) > 0 && proj_len(i) < 1 && p(i, 3) > z_min && p(i, 3) < z_max
        num_points = num_points + 1;
        if num_points > size(points, 1)
            points = [points; zeros(max_points, 3)]; %#ok<AGROW>
        end
        points(num_points, :) = p(i, :);
    end
end

% Trim and remove duplicates
points = points(1:num_points, :);
if num_points > 1
    points = unique(points, 'rows', 'stable');
end
end

function in = isPointInConvexPolygon(p, vertices)
% Fast point-in-convex-polygon test using half-plane checks
n = size(vertices, 1) - 1;
in = true;
for i = 1:n
    v1 = vertices(i, :);
    v2 = vertices(i+1, :);
    edge = v2 - v1;
    normal = [edge(2), -edge(1)];
    if dot(normal, p - v1) > 1e-10
        in = false;
        return;
    end
end
end
