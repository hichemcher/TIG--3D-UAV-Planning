function plot_prism(prism, index, show_index, highlight_id)
% ------------------------------------------------------------
% Plots a single 3D polygonal prism obstacle
% ------------------------------------------------------------

vertices = prism.vertices;
z_min = prism.z_range(1);
z_max = prism.z_range(2);

% Close polygon if needed
if ~isequal(vertices(1,:),vertices(end,:))
    vertices(end+1,:) = vertices(1,:);
end

x = vertices(:,1);
y = vertices(:,2);
n = length(x);

% Colors
if index == highlight_id
    base_color = [1.0 0.5 0.0];
    top_color  = [1.0 0.6 0.2];
else
    base_color = [0.3 0.3 0.3];
    top_color  = [0.5 0.5 0.5];
end

edge_color = [0 0 0];

% Bottom
patch(x,y,z_min*ones(n,1),base_color,...
    'EdgeColor',edge_color,...
    'FaceAlpha',0.95,...
    'HandleVisibility','off');

% Top
patch(x,y,z_max*ones(n,1),top_color,...
    'EdgeColor',edge_color,...
    'FaceAlpha',0.95,...
    'HandleVisibility','off');

% Sides
for i = 1:n-1
    xs = [x(i) x(i+1) x(i+1) x(i)];
    ys = [y(i) y(i+1) y(i+1) y(i)];
    zs = [z_min z_min z_max z_max];
    patch(xs,ys,zs,base_color,...
        'EdgeColor',edge_color,...
        'FaceAlpha',0.95,...
        'HandleVisibility','off');
end

% Index label
if show_index
    centroid = mean(vertices(1:end-1,:),1);
    text(centroid(1),centroid(2),z_max+5,...
        sprintf('%d',index),...
        'Color','y','FontWeight','bold');
end
end
