function metrics = compute_path_metrics(path, ds)

metrics.length = computePathLength(path);
[metrics.totalTurningAngle, metrics.sharpTurns] = computeTurningAngle(path);
metrics.maxHeadingChange = computeMaxHeadingChange(path, ds);

end


%% ================= LENGTH =================
function L = computePathLength(path)

L = 0;
for i = 1:size(path,1)-1
    L = L + norm(path(i,:) - path(i+1,:));
end

end


%% ================= TURNING =================
function [totalAngle, sharpTurns] = computeTurningAngle(path)

totalAngle = 0;
sharpTurns = 0;

if size(path,1) < 3
    return
end

for i = 2:size(path,1)-1
    
    A = path(i-1,:);
    B = path(i,:);
    C = path(i+1,:);
    
    v1 = A - B;
    v2 = C - B;
    
    if norm(v1)==0 || norm(v2)==0
        continue
    end
    
    cosTheta = dot(v1,v2)/(norm(v1)*norm(v2));
    cosTheta = max(-1,min(1,cosTheta));
    
    angle = pi - acos(cosTheta);
    
    totalAngle = totalAngle + angle;
    
    if rad2deg(angle) > 90
        sharpTurns = sharpTurns + 1;
    end
end

end


%% ================= HEADING =================
function maxDelta = computeMaxHeadingChange(path, ds)

resampled = resamplePath(path, ds);
theta = headingAngles(resampled);

dtheta = abs(diff(theta));
dtheta = abs(unwrap(dtheta));

maxDelta = max(dtheta);

end


%% ================= RESAMPLE =================
function P = resamplePath(path, ds)

P = path(1,:);

for i = 1:size(path,1)-1
    d = norm(path(i+1,:) - path(i,:));
    n = max(1, ceil(d/ds));
    
    for k = 1:n
        t = k/n;
        P = [P; (1-t)*path(i,:) + t*path(i+1,:)];
    end
end

end


%% ================= HEADING ANGLES =================
function theta = headingAngles(path)

dP = diff(path(:,1:2));
theta = atan2(dP(:,2), dP(:,1));

end
