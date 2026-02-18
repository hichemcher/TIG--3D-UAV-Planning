

function [dynamic_path, tig_time] = otig3d_unknown_planner( ...
    start, goal, mapSize, obstacles_raw,  safety_distance,range)
% -------------------------------------------------------------------------
% OTIG*3D_UNKNOWN_PLANNER
% -------------------------------------------------------------------------
% Online 3D Tangent Intersection Guidance (O-TIG*-3D) Planner
% for Unknown Environments
%
% This planner performs iterative local planning using OTIG within a
% limited sensing radius. At each iteration:
%   1) Obstacles inside sensing range are extracted
%   2) Local OTIG planning is executed
%   3) The agent advances along the computed sub-path
%   4) Sensing region is recentered
%
% If local planning fails:
%   • Sensing radius is gradually expanded
%   • A forced motion step is applied as escape behavior
%
% Inputs:
%   start       : [x y z] initial position
%   goal        : [x y z] goal position
%   obstacles   : obstacle structure array
%   mapSize   : environment dimensions [X Y Z]
%   range       : sensing structure (x, y, z, radius)
%
% Outputs:
%   dynamic_path : executed trajectory (Nx3)
%   tig_time     : total computation time (seconds)
% -------------------------------------------------------------------------

%% ===================== Preprocessing =====================

% Inflate obstacles to ensure safety margin
obstacles = inflate_obstacles(obstacles_raw, safety_distance);


%% ===================== Initialization =====================

% Initial node definition
P1.coor    = start;
P1.prev    = [];
P1.prevobs = -1;
P1.currobs = -1;
P1.istop   = false;
P1.org     = start;

% Goal node
P2.coor  = goal;
P2.org   = [];
P2.istop = false;

% Output path
dynamic_path = start;
current = P1;

% Timing
tic_handle = tic;
tig_time   = 0;

% Sensing expansion parameters
maxExpand = 5;                 % maximum expansion attempts
expandCnt = 0;
R_INIT    = range.radius;      % initial sensing radius
R_MAX     = 200;               % maximum sensing radius

% Environment bounds (for visualization/debug if needed)
bounds = [0 mapSize(1) ...
          0 mapSize(2) ...
          0 mapSize(3)];

%% ===================== Main Replanning Loop =====================

while norm(goal - current.coor) > 0.1

    % ---------------------------------------------------------
    % Extract obstacles within sensing radius
    % ---------------------------------------------------------
    obstacles_local = obstacles_in_range( ...
        current.coor, obstacles, range.radius);

    % ---------------------------------------------------------
    % Local OTIG Planning
    % ---------------------------------------------------------
    sub_path = OTIG_Planning(P1, P2, ...
                             obstacles_local, ...
                             range, ...
                             mapSize);

    % ---------------------------------------------------------
    % Case 1: Local Planning Failure
    % ---------------------------------------------------------
    if isempty(sub_path)

        % Expand sensing radius (bounded expansion)
        if expandCnt < maxExpand && range.radius < R_MAX
            range.radius = min(range.radius * 1.2, R_MAX);
            expandCnt = expandCnt + 1;
            continue;
        end

        % -----------------------------------------------------
        % Escape Behavior (Forced Motion)
        % -----------------------------------------------------
        step_size = 10;  % motion step (meters)

        direction = goal - current.coor;
        direction = direction / norm(direction);

        next_pos = current.coor + step_size * direction;

        % Commit forced motion
        dynamic_path = [dynamic_path; next_pos];

        current.coor = next_pos;
        P1 = current;

        % Reset sensing parameters
        range.radius = R_INIT;
        expandCnt = 0;

        range.x = next_pos(1);
        range.y = next_pos(2);
        range.z = next_pos(3);

        continue;
    end

    % Reset expansion counter after successful planning
    expandCnt = 0;

    % ---------------------------------------------------------
    % Case 2: Valid Sub-Path Found
    % ---------------------------------------------------------
    sub_path = fliplr(sub_path);

    for i = 1:numel(sub_path)
        dynamic_path = [dynamic_path; sub_path(i).coor];
    end

    % Update current state
    P1.prev = current;
    current = sub_path(end);
    P1 = current;

    % Recenter sensing region
    range.x = current.coor(1);
    range.y = current.coor(2);
    range.z = current.coor(3);

    tig_time = toc(tic_handle);
end

end





function path = OTIG_Planning(start, goal, obstacles, range, mapSize)
% -------------------------------------------------------------------------
% OTIG*_PLANNING
% -------------------------------------------------------------------------
% Dynamic Tangent Intersection Guidance (O-TIG*) Planner
%
% This function computes a locally optimal motion segment from a given
% start node toward a goal node within a limited sensing range.
%
% The planner integrates:
%   • Tangent-based waypoint generation
%   • A*-like node expansion
%   • Smoothness (angle) constraints
%   • Obstacle-top traversal logic
%
% Inputs:
%   start       : struct (current node state)
%   goal        : struct (goal node state)
%   obstacles   : obstacle structure array
%   range       : local sensing / planning radius
%   mapSize     : 3D environment dimensions
%
% Output:
%   path        : struct array of nodes (local planned segment)
% -------------------------------------------------------------------------

%% ===================== Initialization =====================

% Start node
startNode.coor     = start.coor;
startNode.tang     = [];
startNode.gScore   = 0;
startNode.fScore   = norm(start.coor - goal.coor);
startNode.prev     = [];
startNode.prevobs  = start.prevobs;
startNode.currobs  = start.currobs;
startNode.istop    = start.istop;

% Goal node
goalNode.coor     = goal.coor;
goalNode.prev     = [];
goalNode.prevobs  = -1;
goalNode.currobs  = -1;
goalNode.tang     = goal;
goalNode.istop    = goal.istop;

% Heuristic (Euclidean distance)
heuristic = @(x, y) norm(x - y);

% Open / Closed sets
openSet   = {startNode};
closedSet = {};

% Path initialization
path = startNode;

% Memory containers
waypoints_history   = goalNode.coor;
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
    % Termination Condition
    % Stop if:
    %   1) Node reaches sensing boundary
    %   2) Goal is reached
    % ---------------------------------------------------------
    if is_point_on_perimiter(current.coor, range) || ...
       isequal(current.coor, goalNode.coor)

        path = current;
        traceNode = current;

        while ~isempty(traceNode.prev)

            if isequal(traceNode.prev.coor, startNode.coor)
                break;
            end

            path(end+1) = traceNode.prev;
            traceNode = traceNode.prev;
        end

        return;
    end

    %% ===================== Neighbor Generation =====================

    waypoints = [];
    current_expanded = current;

    % Previous states for angle constraint
    if ~isempty(current.prev)
        current_expanded.prev = current.prev.coor;

        if ~isempty(current.prev.prev)
            current_expanded.prev2 = current.prev.prev.coor;
        else
            current_expanded.prev2 = current.prev.coor;
        end
    else
        current_expanded.prev  = current.coor;
        current_expanded.prev2 = current.coor;
    end

    % Detect first intersected obstacle
    [obsFirst, obsList] = ...
        get_first_intersected_obstacle(current_expanded.coor, ...
                                      goalNode.coor, ...
                                      obstacles);

    %% ===================== Waypoint Strategy =====================

    if current_expanded.istop && ...
       ~isempty(obsFirst) && ...
       (current_expanded.coor(3) < obsFirst.height + 5) && ...
       isequal(obsFirst.index, current_expanded.currobs)

        waypoint = get_top_waypoint(current_expanded, ...
                                    goalNode.coor, ...
                                    obsFirst, ...
                                    range);

        if ~isnan(waypoint.coor(:,1))
            waypoints = [waypoints waypoint];
        end

    else

        waypoints = generate_waypoints_direct(current_expanded, ...
                                      goalNode, ...
                                      obsList, ...
                                      obstacles, ...
                                      true, ...
                                      range);

        if isempty(waypoints)

            [waypoints, ~, waypoints_history] = ...
                generate_waypoints_explore(current_expanded, ...
                             goalNode, ...
                             obstacles, ...
                             waypoints_history, ...
                             mapSize,true,range);
        end
    end

    %% ===================== Neighbor Evaluation =====================

    for i = 1:size(waypoints,1)

        neigh = waypoints(i,:);

        neighbor.coor       = neigh.coor;
        neighbor.tang       = neigh.tang;
        neighbor.istop      = neigh.istop;
        neighbor.prev       = current;
        neighbor.prevobs    = neigh.prevobs;
        neighbor.currobs    = neigh.currobs;
        neighbor.gScore     = inf;
        neighbor.fScore     = inf;

        % Skip if already explored
        if isContainClosed(closedSet, neighbor.coor)
            continue;
        end

        % Previous node for curvature constraint
        if ~isempty(current.prev)
            neighbor.prev2 = current.prev.coor;
        else
            neighbor.prev2 = current.coor;
        end

        % ---------------------------------------------------------
        % Angle Constraint (UAV smoothness condition)
        % ---------------------------------------------------------
        turnAngle = calculate_angle(neighbor.prev2, ...
                                   neighbor.prev.coor, ...
                                   neighbor.coor);

        if turnAngle < 60 || turnAngle > 300
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
                              heuristic(neighbor.coor, goalNode.coor);

            % Avoid duplicate insertion (2D comparison)
            existsInOpen = any(cellfun(@(node) ...
                isequal(node.coor(1:2), neighbor.coor(1:2)), openSet));

            if ~existsInOpen
                openSet{end+1} = neighbor;
            end
        end
    end
end

%% ===================== No Path Case =====================

path = [];

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












