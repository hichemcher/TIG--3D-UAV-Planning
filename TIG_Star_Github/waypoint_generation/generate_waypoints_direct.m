function [waypoints, tangents] = ...
    generate_waypoints_direct(P1, P2, obslistFirst, obstacles, useRange, range)

waypoints = [];
tangents  = [];

for i = 1:length(obslistFirst)

    obstacle = obslistFirst{i};

    % -------- Compute Tangents --------
    [wp1, wp2, wpTop] = compute_tangents(P1.coor, P2.coor, obstacle);

    candidates = {wp1, wp2, wpTop};
    isTop      = [false false true];

    for k = 1:3

        candidate = candidates{k};

        if isempty(candidate), continue; end
        if any(isnan(candidate(1,:))), continue; end

        candidate = candidate(1,:);

        % -------- ANGLE FILTER --------
        if ~isempty(P1.prev)
            ang = calculate_angle(candidate, P1.coor, P1.prev);
            if ang < 60 || ang > 300
                continue;
            end
        end

        % -------- INTERSECTION CHECK --------
        [obs_int,~,~] = ...
            get_first_intersected_obstacle(P1.coor, candidate, obstacles);

        if ~isempty(obs_int)
            continue;
        end

        % -------- BUILD STRUCT --------
        tn_struct.coor      = candidate;
        tn_struct.currobs   = obstacle.index;
        tn_struct.prevobs   = P1.currobs;
        tn_struct.tang      = [];
        tn_struct.searchCost= [];
        tn_struct.istop     = isTop(k);

        % -------- RANGE LIMIT (OPTIONAL) --------
        if useRange
            if point_out_range(candidate, range)
                temp = calculate_inrange_waypoint(P1, tn_struct, range);
                tn_struct.coor = temp.coor;
            end
        end

        tangents  = [tangents; candidate];
        waypoints = [waypoints; tn_struct];

    end
end
end
