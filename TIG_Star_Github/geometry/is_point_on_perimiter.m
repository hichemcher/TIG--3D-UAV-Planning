function result = is_point_on_perimiter(node, range)
x = node(1);
y = node(2);
z = node(3);
center_x = range.x;
center_y = range.y;
center_z = range.z;
radius = range.radius;

    distance = sqrt((x - center_x)^2 + (y - center_y)^2 + (z-center_z)^2);
    result = ismembertol(distance, radius);
end
