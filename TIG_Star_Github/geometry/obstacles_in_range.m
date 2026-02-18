function in_range_indices = obstacles_in_range(uav_pos, polygons, r)
    % polygons: array of structs with fields 'vertices' (Nx2) and 'height'
    % uav_pos: 1x3 [x, y, z]
    % r: sensing radius (scalar)
    
    in_range_indices = {};
    
    for i = 1:length(polygons)
        if is_obstacle_in_range(polygons{i}, uav_pos, r)
           
            in_range_indices = [in_range_indices polygons{i}] ; %#ok<AGROW>
        end
    end
end

function in_range = is_obstacle_in_range(polygon, uav_pos, r)
    % UAV position and sensing radius
    x_uav = uav_pos(1);
    y_uav = uav_pos(2);
    z_uav = uav_pos(3);
    
    % Polygon base and height
    base_xy = polygon.vertices;  % Nx2
    z_min = 0;
    z_max = polygon.height;

    % 1. Project UAV to polygon base plane (2D)
    dist_xy = point2poly_distance([x_uav, y_uav], base_xy);

    % 2. Compute vertical (z) distance to polygon
    if z_uav < z_min
        dz = z_min - z_uav;
    elseif z_uav > z_max
        dz = z_uav - z_max;
    else
        dz = 0;
    end

    % 3. Euclidean distance to polygon prism
    d = sqrt(dist_xy^2 + dz^2);

    % 4. Check if within sensing range
    in_range = d <= r;
end

function d = point2poly_distance(p, poly)
    % Compute shortest distance from point to 2D polygon
    if inpolygon(p(1), p(2), poly(:,1), poly(:,2))
        d = 0;
    else
        % Distance to polygon edges
        d = inf;
        n = size(poly,1);
        for i = 1:n
            v1 = poly(i,:);
            v2 = poly(mod(i,n)+1,:);
            d = min(d, point2segment_dist(p, v1, v2));
        end
    end
end

function d = point2segment_dist(p, a, b)
    % Distance from point p to segment ab
    ap = p - a;
    ab = b - a;
    t = dot(ap, ab) / dot(ab, ab);
    t = max(0, min(1, t));
    proj = a + t * ab;
    d = norm(p - proj);
end
