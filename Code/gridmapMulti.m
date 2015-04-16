addpath('tools')
mkdir plots
more off
close all
clear all

% myCluster = parcluster('local');
% myCluster.NumWorkers = 2;
% saveAsProfile(myCluster,'local');
% matlabpool(myCluster.NumWorkers);


% Load laser scans and robot poses.
load('../Data/albert.mat')

nRobots=4;
[data,mT]=partitionDataAlbert(nRobots,pose,odom,rad);


% Initial cell occupancy probability.
probPrior = 0.50;
% Probabilities related to the laser range finder sensor model.
probOcc = 0.9;
probFree = 0.35;

% Map grid size in meters. Decrease for better resolution.
gridSize = 0.1;

% Set up map boundaries and initialize map.
border = 50;
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
map = logOddsPrior*ones([mapSize nRobots]);
mapCombined = logOddsPrior*ones([mapSize nRobots]);
disp('Map initialized. Map size:'), disp(size(map))

% Map offset used when converting from world to map coordinates.
offset = [offsetX; offsetY];


% Main loop for updating map cells.
% You can also take every other point when debugging to speed up the loop (t=1:2:size(poses,1))
for t=1:mT
    t
    % Robot pose at time t.
    %parfor a1=1:nRobots
    for a1=1:nRobots
        if (size(data(a1).pose,2)>=t)
            robPose=data(a1).pose(:,t);

            % Laser scan made at time t.
            sc=data(a1).rad(:,t);

            
            
            % Compute the mapUpdate, which contains the log odds values to add to the map.
            [mapUpdate, robPoseMapFrame(:,:,a1), laserEndPntsMapFrameInter] = inv_sensor_model(map(:,:,a1), sc, robPose, gridSize, offset, probPrior, probOcc, probFree);
            laserEndPntsMapFrame{a1}=laserEndPntsMapFrameInter;
            % TODO: Update the occupancy values of the map cells.
            map(:,:,a1)=map(:,:,a1)+mapUpdate;
            
        end
        
    end
    
    mapCombined=sum(map,3);
    % Plot current map and robot trajectory so far.
    plot_map_multi(mapCombined, mapBox, robPoseMapFrame, data, laserEndPntsMapFrame, gridSize, offset, t);
end


system(sprintf('avconv -r 5 -b 0.5M -i plots/gridmap_%%03d.png %s-gridmap.mp4',datestr(now,30)))
% matlabpool('close');