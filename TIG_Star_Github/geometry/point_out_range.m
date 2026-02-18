function outside = point_out_range(node, range)
x = node(1);
y = node(2);
z = node(3);
    cx = range.x;
    cy = range.y;
    cz = range.z;
    radius = range.radius;
    distance = sqrt((x - cx)^2 + (y - cy)^2 + (z-cz)^2);
    if distance > radius
        outside = true;
    else
        outside = false;
    end
end