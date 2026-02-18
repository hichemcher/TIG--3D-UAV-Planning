% ------------------------------------------------------------
% TIG*3D Planner
%
% Author: H. Cheriet
% Affiliation: USTO-MB 
% Year: 2026
% ------------------------------------------------------------


clc; clear; close all;

% ------------------------------------------------------------
% Setup Paths
% ------------------------------------------------------------
currentFolder = fileparts(mfilename('fullpath'));
projectRoot   = fileparts(currentFolder);
addpath(genpath(projectRoot));

% ------------------------------------------------------------
% Load Environment
% ------------------------------------------------------------
load(fullfile(projectRoot, "maps", "short1.mat"));

safety_distance = 2;  % meters

% ------------------------------------------------------------
% Run Planner
% ------------------------------------------------------------
[path, elapsedTime] = stig3d_planner(start, goal, mapSize, polygons, safety_distance);
fprintf('Computation Time: %.4f s\n', elapsedTime);

% ------------------------------------------------------------
% Metrics
% ------------------------------------------------------------
ds = 1.0;
metrics = compute_path_metrics(path, ds);

disp('--- Static TIG* ---')
disp(metrics)

% ------------------------------------------------------------
% Visualization
% ------------------------------------------------------------
env.start     = start;
env.goal      = goal;
env.polygons  = polygons;
env.mapSize   = mapSize;

results.path           = path;
results.path_smoothed  = [];
results.algorithm_name = "S-TIG*";

options.show_index        = false;
options.highlight_obstacle = -1;

display_environment_3d(env, results, options);

% ------------------------------------------------------------
% Animate UAV Motion
% ------------------------------------------------------------
anim_options.speed = 1;   % smaller = faster
anim_options.trail = false;   % leave red trail

%animate_path(path, anim_options);
