function waypoint = calculate_inrange_waypoint(waypoint,current_point,range)


    [x,y,z] = lineCircleIntersection(current_point.coor, waypoint.coor,range);
    %disp(["x and y",num2str([x,y])])
    %disp([current_point.coor,waypoint.coor,[range.x range.y]])
    if(length(x)==2)
        d1 = norm([x(1) y(1) z(1)]-waypoint.coor);
        d2 = norm([x(2) y(2) z(2)]-waypoint.coor);
        if(d1<d2)
            waypoint.coor =[x(1),y(1),z(1)] ;
            
        else
            waypoint.coor =[x(2),y(2),z(2)] ;

        end
    else
        waypoint.coor =[x,y,z] ;
    end
    waypoint.tang = [-1 -1 -1];

    waypoint.istop = current_point.istop;
    waypoint.currobs = current_point.currobs;

end
function [x_intersect, y_intersect z_intersect] = lineCircleIntersection(A, B,range)
    % Calculate direction vector of the line segment
    h = range.x;
    k = range.y;
    v = range.z;
    r = range.radius;
    x1 = A(1);
    x2 = B(1);
    y1 = A(2);
    y2 = B(2);
    z1 = A(3);
    z2 = B(3);
     dx = x2 - x1;
    dy = y2 - y1;
    dz = z2 - z1;
    % Calculate parameters for quadratic equation
    A = dx^2 + dy^2 + dz^2;
    B = 2 * (dx * (x1 - h) + dy * (y1 - k) + + dz * (z1 - v));
    C = (x1 - h)^2 + (y1 - k)^2 + (z1 - v)^2 - r^2;

    % Calculate discriminant
    discriminant = B^2 - 4 * A * C;

    if discriminant < 0
        % No intersection
        x_intersect = [];
        y_intersect = [];
        z_intersect = [];
    elseif discriminant == 0
        % One intersection
        t = -B / (2 * A);
        x_intersect = x1 + t * dx;
        y_intersect = y1 + t * dy;
        z_intersect = z1 + t * dz;
        if ~isBetween(x_intersect, x1, x2) || ~isBetween(y_intersect, y1, y2) || ~isBetween(z_intersect, z1, z2)
            % Intersection point lies outside the line segment
            x_intersect = [];
            y_intersect = [];
             z_intersect = [];
        end
    else
        % Two intersections
        t1 = (-B + sqrt(discriminant)) / (2 * A);
        t2 = (-B - sqrt(discriminant)) / (2 * A);
        x_intersect = [x1 + t1 * dx, x1 + t2 * dx];
        y_intersect = [y1 + t1 * dy, y1 + t2 * dy];
        z_intersect = [z1 + t1 * dz, z1 + t2 * dz];
        % Filter out points outside the line segment
        outside_idx = ~isBetween(x_intersect, x1, x2) | ~isBetween(y_intersect, y1, y2)| ~isBetween(z_intersect, z1, z2);
        x_intersect(outside_idx) = [];
        y_intersect(outside_idx) = [];
         z_intersect(outside_idx) = [];
    end
end
function inside = isBetween(value, lowerBound, upperBound)
    inside = value >= min(lowerBound, upperBound) & value <= max(lowerBound, upperBound);
end