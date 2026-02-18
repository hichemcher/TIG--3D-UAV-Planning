function [path, path_time] = stig3d_planner(start, goal, mapSize, obstacles_raw, safety_distance)
% -------------------------------------------------------------------------
% TIG*3D Planner
%
% Author: H. Cheriet
% Affiliation: USTO-MB 
% Year: 2026
% -------------------------------------------------------------------------
% 3D Static Tangent Intersection Guidance Star (S-TIG*-3D) Planner
%
% This function computes a collision-free path between start and goal
% positions in a 3D environment containing inflated obstacles.
% -------------------------------------------------------------------------

%% ===================== Preprocessing =====================

% Inflate obstacles to ensure safety margin
obstacles = inflate_obstacles(obstacles_raw, safety_distance);

% Start timing
timer_handle = tic;

% Heuristic (Euclidean distance)
heuristic = @(x, y) norm(x - y);

%% ===================== Node Initialization =====================

% Start node
startNode.coor     = start;
startNode.tang     = [];
startNode.gScore   = 0;
startNode.fScore   = heuristic(start, goal);
startNode.prev     = [];
startNode.prevobs  = -1;
startNode.currobs  = -1;
startNode.istop    = false;

% Goal node
goalNode.coor     = goal;
goalNode.prev     = [];
goalNode.prevobs  = -1;
goalNode.currobs  = -1;
goalNode.tang     = goal;
goalNode.istop    = false;

%% ===================== A*-like Initialization =====================

openSet   = {startNode};
closedSet = {};

path = [];
waypoints_history = goalNode.coor;
calculated_tangents = [];

%% ===================== Main Search Loop =====================

while ~isempty(openSet)

    % ---------------------------------------------------------
    % Select node with minimum fScore
    % ---------------------------------------------------------
    fScores = cellfun(@(node) node.fScore, openSet);
    [~, min_index] = min(fScores);

    current = openSet{min_index};
    openSet(min_index) = [];

    closedSet{end+1} = current;

    % ---------------------------------------------------------
    % Goal Check
    % ---------------------------------------------------------
    if isequal(current.coor, goal)

        path = goal;
        node_trace = current;

        while ~isempty(node_trace.prev)
            path = [path; node_trace.prev.coor];
            node_trace = node_trace.prev;
        end

        path_time = toc(timer_handle);
        return;
    end

    %% ===================== Neighbor Generation =====================

    waypoints = [];

    current_expanded = current;

    if ~isempty(current.prev)
        current_expanded.prev  = current.prev.coor;

        if ~isempty(current.prev.prev)
            current_expanded.prev2 = current.prev.prev.coor;
        else
            current_expanded.prev2 = current.prev.coor;
        end
    else
        current_expanded.prev  = current.coor;
        current_expanded.prev2 = current.coor;
    end

    % Detect first intersected obstacle along direct line
    [obsFirst, obsList] = ...
        get_first_intersected_obstacle(current_expanded.coor, ...
                                       goalNode.coor, obstacles);

    % ---------------------------------------------------------
    % Waypoint Strategy
    % ---------------------------------------------------------
    if current_expanded.istop && ...
       ~isempty(obsFirst) && ...
       isequal(obsFirst.index, current_expanded.currobs)

        waypoint = get_top_waypoint(current_expanded, goalNode.coor, obsFirst);

        if ~isnan(waypoint.coor(:,1))
            waypoints = [waypoints waypoint];
        end

    else

        waypoints = generate_waypoints_direct(current_expanded, ...
                                              goalNode, ...
                                              obsList, ...
                                              obstacles,false,[]);

        if isempty(waypoints)
            [waypoints, ~, waypoints_history] = ...
                generate_waypoints_explore(current_expanded, ...
                                           goalNode, ...
                                           obstacles, ...
                                           waypoints_history, ...
                                           mapSize,false,[]);
        end
    end

    %% ===================== Neighbor Evaluation =====================

    for i = 1:size(waypoints,1)

        neighbor_raw = waypoints(i,:);

        neighbor.coor     = neighbor_raw.coor;
        neighbor.tang     = neighbor_raw.tang;
        neighbor.istop    = neighbor_raw.istop;
        neighbor.prev     = current;
        neighbor.prevobs  = neighbor_raw.prevobs;
        neighbor.currobs  = neighbor_raw.currobs;
        neighbor.gScore   = inf;
        neighbor.fScore   = inf;

        % Skip if already explored
        if isContainClosed(closedSet, neighbor.coor)
            continue;
        end

        % Compute second previous node (for angle constraint)
        if ~isempty(current.prev)
            neighbor.prev2 = current.prev.coor;
        else
            neighbor.prev2 = current.coor;
        end

        % ---------------------------------------------------------
        % Angle Constraint (Smooth UAV motion constraint)
        % ---------------------------------------------------------
        turn_angle = calculate_angle(neighbor.prev2, ...
                                     neighbor.prev.coor, ...
                                     neighbor.coor);

        if turn_angle < 60 || turn_angle > 300
            continue;
        end

        % ---------------------------------------------------------
        % Cost Update
        % ---------------------------------------------------------
        tentative_gScore = current.gScore + ...
                           heuristic(current.coor, neighbor.coor);

        if tentative_gScore < neighbor.gScore

            neighbor.gScore = tentative_gScore;
            neighbor.fScore = tentative_gScore + ...
                              heuristic(neighbor.coor, goal);

            % Avoid duplicate insertion (2D coordinate comparison)
            existsInOpen = any(cellfun(@(node) ...
                isequal(node.coor(1:2), neighbor.coor(1:2)), openSet));

            if ~existsInOpen
                openSet{end+1} = neighbor;
            end
        end
    end
end

%% ===================== No Path Case =====================

disp('No path found.');
path = [];
path_time = toc(timer_handle);

end

function contains = isContainClosed(cellArray, element)
% Check if any struct in cellArray has coor(1:2) equal to element(1:2)
contains = false;

for i = 1:numel(cellArray)
    item = cellArray{i};
    if isfield(item, 'coor') && isequal(item.coor(1:2), element(1:2))
        contains = true;
        break;
    end
end
end












