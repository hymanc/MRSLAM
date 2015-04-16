addpath('tools')
mkdir plots
more off
close all
%clear all

% Load laser scans and robot poses.
load('../Data/albert.mat')

% Initial cell occupancy probability.
probPrior = 0.50;
% Probabilities related to the laser range finder sensor model.
probOcc = 0.9;
probFree = 0.35;

% Map grid size in meters. Decrease for better resolution.
gridSize = 0.1;

% Set up map boundaries and initialize map.
border = 30;
robXMin = min(pose(1,:));
robXMax = max(pose(1,:));
robYMin = min(pose(2,:));
robYMax = max(pose(2,:));
mapBox = [robXMin-border robXMax+border robYMin-border robYMax+border];
offsetX = mapBox(1);
offsetY = mapBox(3);
mapSizeMeters = [mapBox(2)-offsetX mapBox(4)-offsetY];
mapSize = ceil([mapSizeMeters/gridSize]);

% Used when updating the map. Assumes that prob_to_log_odds.m
% has been implemented correctly.
logOddsPrior = prob_to_log_odds(probPrior);

% The occupancy value of each cell in the map is initialized with the prior.
map = logOddsPrior*ones(mapSize);
disp('Map initialized. Map size:'), disp(size(map))

% Map offset used when converting from world to map coordinates.
offset = [offsetX; offsetY];

% Main loop for updating map cells.
% You can also take every other point when debugging to speed up the loop (t=1:2:size(poses,1))
for t=1:size(pose,2)
    t
    % Robot pose at time t.
    robPose=pose(:,t);

    % Laser scan made at time t.
    sc=rad(:,t);

    % Compute the mapUpdate, which contains the log odds values to add to the map.
    [mapUpdate, robPoseMapFrame, laserEndPntsMapFrame] = inv_sensor_model(map, sc, robPose, gridSize, offset, probPrior, probOcc, probFree);
    
    % TODO: Update the occupancy values of the map cells.
    map=map+mapUpdate;
    
    % Plot current map and robot trajectory so far.
    plot_map(map, mapBox, robPoseMapFrame, pose', laserEndPntsMapFrame, gridSize, offset, t);
end


system(sprintf('avconv -r 5 -b 0.5M -i plots/gridmap_%%03d.png %s-gridmap.mp4',datestr(now,30)))
