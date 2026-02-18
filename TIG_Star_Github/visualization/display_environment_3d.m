function display_environment_3d(env, results, options)
% ------------------------------------------------------------
% Function: display_environment_3d
%
% Description:
% Visualizes a 3D UAV environment with polygonal prism obstacles
% and planned paths.
%
% Author: H. Cheriet
% ------------------------------------------------------------

figure;
axis equal;
hold on;
grid on;
view(45,30);

start     = env.start;
goal      = env.goal;
polygons  = env.polygons;
mapSize   = env.mapSize;

path           = results.path;
path_smoothed  = results.path_smoothed;
algoName       = results.algorithm_name;

show_index   = options.show_index;
highlight_id = options.highlight_obstacle;

%% ---- Ground ----
[X,Y] = meshgrid(1:mapSize(2),1:mapSize(1));
surf(X,Y,zeros(size(X)), ...
    'FaceColor',[0.3 0.3 0.3], ...
    'EdgeColor','none', ...
    'FaceAlpha',0.5, ...
    'HandleVisibility','off');

%% ---- Obstacles ----
for k = 1:length(polygons)
    plot_prism(polygons{k}, k, show_index, highlight_id);
end

%% ---- Start & Goal ----
plot3(start(1),start(2),start(3), ...
    'go','MarkerSize',10,'MarkerFaceColor','g', ...
    'DisplayName','Start');

plot3(goal(1),goal(2),goal(3), ...
    'bo','MarkerSize',10,'MarkerFaceColor','b', ...
    'DisplayName','Goal');

text(start(1),start(2),start(3)+0.2,'S', ...
    'FontSize',14,'Color','g','FontWeight','bold');

text(goal(1),goal(2),goal(3)+0.2,'T', ...
    'FontSize',14,'Color','b','FontWeight','bold');

%% ---- Paths ----
if ~isempty(path)
    plot3(path(:,1),path(:,2),path(:,3), ...
        'r-','LineWidth',3, ...
        'DisplayName',sprintf('%s Path',algoName));
end

if ~isempty(path_smoothed)
    plot3(path_smoothed(:,1),path_smoothed(:,2),path_smoothed(:,3), ...
        'b-','LineWidth',3, ...
        'DisplayName','Smoothed Path');
end

%% ---- Axes & Legend ----
xlabel('X');
ylabel('Y');
zlabel('Z');

legend('show','Location','best');   

xlim([0 mapSize(1)]);
ylim([0 mapSize(2)]);
zlim([0 mapSize(3)]);

set(gcf,'Color',[0.95 0.95 0.95]);
set(gca,'Color',[0.8 0.9 1]);

hold off;
end
