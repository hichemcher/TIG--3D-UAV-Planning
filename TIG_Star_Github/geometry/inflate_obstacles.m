function inflated_obstacles = inflate_obstacles(obstacles, safety_distance)
% ------------------------------------------------------------
% Inflates polygonal prism obstacles by safety_distance
% in XY plane and Z direction.
% ------------------------------------------------------------

inflated_obstacles = obstacles;

for i = 1:length(obstacles)

    prism = obstacles{i};
    vertices = prism.vertices;

    % --- Inflate polygon in XY ---
    inflated_vertices = inflate_polygon(vertices, safety_distance);

    % --- Inflate in Z ---
    z_min = prism.z_range(1) - safety_distance;
    z_max = prism.z_range(2) + safety_distance;

    prism.vertices = inflated_vertices;
    prism.z_range  = [z_min z_max];
    prism.height   = z_max;

    inflated_obstacles{i} = prism;
end
end

function inflated_vertices = inflate_polygon(vertices, d)
% ------------------------------------------------------------
% Inflates convex polygon outward by distance d
% ------------------------------------------------------------

centroid = mean(vertices,1);
inflated_vertices = zeros(size(vertices));

for i = 1:size(vertices,1)
    direction = vertices(i,:) - centroid;
    direction = direction / norm(direction + eps);
    inflated_vertices(i,:) = vertices(i,:) + d * direction;
end
end
