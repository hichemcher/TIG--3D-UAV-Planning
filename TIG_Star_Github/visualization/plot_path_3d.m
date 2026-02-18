function plot_path_3d(path,color,label_name)
% ------------------------------------------------------------
% Plots a 3D path
% ------------------------------------------------------------

plot3(path(:,1),path(:,2),path(:,3),...
    'Color',color,...
    'LineWidth',3,...
    'DisplayName',label_name);
end
