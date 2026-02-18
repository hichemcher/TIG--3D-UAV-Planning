function animate_path(path, options)
% ------------------------------------------------------------
% Function: animate_path
%
% Description:
% Animates UAV motion along a 3D path.
%
% Inputs:
%   path            - Nx3 path
%   options.speed   - pause duration (default = 0.05)
%   options.trail   - true/false (default = false)
%
% Author: H. Cheriet
% ------------------------------------------------------------

% ---- Safety Checks ----
if isempty(path) || size(path,2) ~= 3
    warning('animate_path: Invalid or empty path.');
    return;
end

% ---- Default Options ----
if nargin < 2
    options = struct;
end

if ~isfield(options,'speed')
    options.speed = 0.05;
end

if ~isfield(options,'trail')
    options.trail = false;
end

% Ensure we animate on current axes
ax = gca;
hold(ax,'on');

% ---- Create UAV marker once ----
uav = plot3(ax, ...
    path(1,1), path(1,2), path(1,3), ...
    'o', ...
    'MarkerSize',8, ...
    'MarkerFaceColor','r', ...
    'MarkerEdgeColor','k', ...
    'HandleVisibility','off');

% ---- Optional Trail ----
if options.trail
    trail = plot3(ax, ...
        path(1,1), path(1,2), path(1,3), ...
        'r-', ...
        'LineWidth',2, ...
        'HandleVisibility','off');
end

% ---- Animation Loop ----
for i = 2:size(path,1)

    % Update UAV position
    set(uav, ...
        'XData', path(i,1), ...
        'YData', path(i,2), ...
        'ZData', path(i,3));

    % Update trail
    if options.trail
        set(trail, ...
            'XData', path(1:i,1), ...
            'YData', path(1:i,2), ...
            'ZData', path(1:i,3));
    end

    drawnow limitrate;
    pause(options.speed);
end
end
