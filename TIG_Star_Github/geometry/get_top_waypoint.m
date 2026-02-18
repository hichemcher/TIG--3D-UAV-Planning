
function waypoint = get_top_waypoint(current,goal,obstacle)

waypoints = line_polygon_intersection(current.coor(:,1:2), goal(:,1:2), obstacle.vertices);
if size(waypoints,1)>0
    waypoint.coor = [waypoints(1,:) obstacle.height+6];
else
    waypoint.coor = [NaN NaN NaN];
end
waypoint.currobs = obstacle;
waypoint.prevobs = current.currobs;
waypoint.tang = [];
waypoint.istop = false;
waypoint.searchCost = [];
waypoint.prev = current.coor;
waypoint.prev2 = current.prev;
end



function points = line_polygon_intersection(P0, D, vertices)
% Finds intersection points between a segment (P0 to P0+D) and a 2D polygon.
% Inputs:
%   P0       - 1x2 starting point of the segment
%   D        - 1x2 vector from P0 to endpoint (direction * length)
%   vertices - Nx2 polygon vertices (closed polygon not required)
% Output:
%   points   - Mx2 intersection points
extension_dist = 0;
P1 = P0 + D;
points = [];

N = size(vertices, 1);
for i = 1:N
    v1 = vertices(i, :);
    v2 = vertices(mod(i, N) + 1, :);  % wrap around to first

    [intersect, pt] = segment_intersect(P0, P1, v1, v2);
    if intersect
        dir = pt - P0;
        dir = dir / norm(dir + eps);
        pt_extended = pt + extension_dist * dir;
        points(end+1, :) = pt_extended;
    end
end

% Remove duplicates
points = unique(points, 'rows');

    function [intersect, pt] = segment_intersect(p1, p2, q1, q2)
        % Returns true if line segments [p1-p2] and [q1-q2] intersect, and the point

        intersect = false;
        pt = [NaN, NaN];

        r = p2 - p1;
        s = q2 - q1;
        rxs = cross2d(r, s);
        qp = q1 - p1;
        qpxr = cross2d(qp, r);

        if abs(rxs) < 1e-10  % lines are parallel
            return;
        end

        t = cross2d(qp, s) / rxs;
        u = cross2d(qp, r) / rxs;

        epsilon = 1e-8;
        if t > epsilon && t < 1 - epsilon && u >= 0 && u <= 1
            pt = p1 + t * r;
            intersect = true;
        end
    end

    function c = cross2d(a, b)
        % 2D cross product (scalar)
        c = a(1)*b(2) - a(2)*b(1);
    end


end


