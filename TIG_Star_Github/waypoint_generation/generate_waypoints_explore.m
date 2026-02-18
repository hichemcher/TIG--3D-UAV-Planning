function [waypoints, tangents, waypoints_traited] = ...
    generate_waypoints_explore(P1, P2, obstacles, waypoints_traited, mapSize, useRange, range)

waypoints = [];
tangents  = [];
explored_waypoints = [];

% ---------------- INITIALIZE STRUCTS ----------------
left_tn_struct  = struct('coor',[],'currobs',-1,'prevobs',-1,'istop',false);
right_tn_struct = struct('coor',[],'currobs',-1,'prevobs',-1,'istop',false);
top_tn_struct   = struct('coor',[],'currobs',-1,'prevobs',-1,'istop',true);

% ---------------- CURRENT POINT ----------------
current_point = P1;
next_point    = P2;

to_explore_coor    = next_point.coor;
to_explore_prevobs = next_point.prevobs;
to_explore_currobs = next_point.currobs;
to_explore_istop   = next_point.istop;

% =========================================================
% ==================== MAIN LOOP ==========================
% =========================================================

while ~isempty(to_explore_coor)

    % -------- POP LAST (DFS style) --------
    next_point.coor    = to_explore_coor(end,:);
    next_point.prevobs = to_explore_prevobs(end,:);
    next_point.currobs = to_explore_currobs(end,:);
    next_point.istop   = to_explore_istop(end,:);

    explored_waypoints = [explored_waypoints; next_point.coor];

    to_explore_coor(end,:)    = [];
    to_explore_prevobs(end,:) = [];
    to_explore_currobs(end,:) = [];
    to_explore_istop(end,:)   = [];

    while true

        % ---------- OBSTACLE CHECK ----------
        [obstacle,~,obs_index] = ...
            get_first_intersected_obstacle(current_point.coor, next_point.coor, obstacles);

        % ==================================================
        % ===== NO INTERSECTION -> VALID SEGMENT ===========
        % ==================================================
        if isempty(obstacle)

            % ---------- RANGE CLIPPING (OPTIONAL) ----------
            if useRange
                if point_out_range(next_point.coor, range)
                    next_point = calculate_inrange_waypoint(next_point,current_point,range);
                end
            end

            waypoints = [waypoints; next_point];
            break;

        else
            % ==================================================
            % ========== OBSTACLE INTERSECTION ================
            % ==================================================

            [L,R,T] = compute_tangents(current_point.coor, next_point.coor, obstacle);

            tangents_candidates = {L,R,T};
            isTop = [false false true];

            for k = 1:3

                candidate = tangents_candidates{k};
                if any(isnan(candidate)), continue; end

                % -------- ANGLE CONSTRAINT --------
                ang = calculate_angle(candidate,current_point.coor,current_point.prev);
                if ang < 60 || ang > 300
                    continue;
                end

                % -------- RANGE LIMIT (OPTIONAL) --------
                if useRange
                    if point_out_range(candidate, range)
                        tempStruct = struct('coor',candidate);
                        tempStruct = calculateInrangeWaypoint(tempStruct,current_point,range);
                        candidate = tempStruct.coor;
                    end
                end

                tangents = [tangents; candidate];

                % -------- DUPLICATE CHECK --------
                if any(all(bsxfun(@eq, explored_waypoints, candidate),2)), continue; end
                if any(all(bsxfun(@eq, to_explore_coor, candidate),2)), continue; end
                if any(all(bsxfun(@eq, waypoints_traited, candidate),2)), continue; end

                % -------- MAP BOUNDS --------
                if any(candidate < 0) || any(candidate > mapSize)
                    continue;
                end

                % -------- PREVENT BACK OBSTACLE LOOP --------
                if ~isequal(current_point.prevobs, obs_index) || isequal(current_point.prevobs,-1)

                    to_explore_coor    = [to_explore_coor; candidate];
                    to_explore_prevobs = [to_explore_prevobs; current_point.currobs];
                    to_explore_currobs = [to_explore_currobs; obs_index];
                    to_explore_istop   = [to_explore_istop; isTop(k)];
                end
            end

            waypoints_traited = [waypoints_traited; current_point.coor];
        end

        break;
    end
end
end
