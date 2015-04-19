% gridmapMultiHoward.m
% Multi-Robot FastSLAM Implementation based on 
% "Multi-robot Simultaneous Localization and Mapping using Particle
% Filters"
% 18 APR 2015
% R. Choroszucha, A. Collier, C. Hyman
% EECS 568
% University of Michigan

addpath('tools')

mkdir plots
more off
close all
%clear all

% Parallel PF execution cluster
myCluster = parcluster('local');
myCluster.NumWorkers = 4;
saveAsProfile(myCluster,'local');
parpool(myCluster.NumWorkers);


% Load laser scans and robot poses.
load('../Data/CustomData-10Robots.mat')

% Noise parameters
alphas = [0.05 0.001 0.005 0.01 0.01 0.01].^2;

% Number of Maps/Particles
nParticles=10;

% Number of robots
nRobots=5;

% Initial cell occupancy probability.
probPrior = 0.50;
% Probabilities related to the laser range finder sensor model.
probOcc = 0.9;
probFree = 0.35;

% Map grid size in meters. Decrease for better resolution.
gridSize = 1;

% Set up map boundaries and initialize map.
border =10;

%%{
%figure(1)
pose=repmat(data(1).pose(:,1),[1 nRobots]);
for a1=1:nRobots
    for a2=2:(size(data(1).pose,2)-1)
        pose(:,a1,a2)=Odometry(data(a1).v(a2),data(a1).omega(a2),dt,pose(:,a1,a2-1));
        robPose(:,a1)=data(a1).pose(:,1);
    end
    %plot(squeeze(pose(1,a1,:)),squeeze(pose(2,a1,:)),'k')
    %hold on;
end
%}

robXMin = min(min(pose(2,:,:)));
robXMax = max(max(pose(2,:,:)))+50;
robYMin = min(min(pose(1,:,:)));
robYMax = max(max(pose(1,:,:)))+50;

mapBox = [robXMin-border robXMax+border robYMin-border robYMax+border];
offsetX = mapBox(1);
offsetY = mapBox(3);
mapSizeMeters = [mapBox(2)-offsetX mapBox(4)-offsetY];
mapSize = ceil(mapSizeMeters/gridSize);

% Used when updating the map. Assumes that prob_to_log_odds.m
% has been implemented correctly.
logOddsPrior = prob_to_log_odds(probPrior);

% The occupancy value of each cell in the map is initialized with the prior.
map = logOddsPrior*ones([mapSize nParticles]); 
mapCombined = logOddsPrior*ones(mapSize);
disp('Map initialized. Map size:'), disp(size(map))

% Map offset used when converting from world to map coordinates.
offset = [offsetX; offsetY];

%% Pre/post encounter queues
joined = [1]; % Joined/Post encounter list (initialize to at least one robot)
aQ = cell(nRobot,1);
cQ = cell(nRobot,1);
% For each enqueued cell (4x1):
% Param 1: actions (u)
% Param 2: measurements (z)
% Param 3: observed robots
% Param 4: observed robot poses

% Main loop for updating map cells.
% You can also take every other point when debugging to speed up the loop (t=1:2:size(poses,1))
robOdom=robPose;
robOdom=repmat(robOdom,[1 1 nParticles]);
robPoseMapFrame=zeros([2 size(data(1).pose,2) nRobots nParticles]);
weight=1/nParticles*ones(nParticles,1); % Initial weights

% TODO: Update until all causal/non-causal data is exhausted
%% Main SLAM loop
for t=1:(size(data(1).pose,2)-1)
    t
    jointTemp = join; % Alias join to enforce wait for newly joined robots
    % Append queues
    for rob = 1:nRobots
        if(size(data(rob).pose,2)>=t)
            update = cell(4,1);
            update{1} = [data(rob).v{t};data(rob).omega{t}]; % u
            update{2} = data(rob).r{t}; % z
            update{3} = 0; % rob
            update{4} = 0; % Delta (relative pose
            for sighting = 1:nRobots % TODO Check for encounter
                if(sighting ~= rob)
                    % TODO: Check if robot is within perceptual radius and not
                    % occluded
                end
            end

            if(find(joined==rob)) % Robot post-encounter, add to causal queue
                cQ{rob} = horzcat(cQ{rob}, update); % Append
            else % Robot before encounter, add to non-causal queue
                aQ{rob} = horzcat(update, aQ{rob}); % Prepend(reverse order)
            end
        end
    end
    
    % Update filter from queues
    for rob = 1:nRobots
        if(find(join == rob)) % Only update joined robots
            dCaus = []; dAcaus = [];
            if(size(cQ{rob},2) >= 1)  % Check causal queue
                dCaus = cQ{rob}(:,1); % Read next data from queue
                cQ{rob}(:,1) = [];    % Dequeue data
            end
            if(size(aQ{rob},2) >= 1) % Check acausal queue
                dAcaus = aQ{rob}(:,1);
                aQ{rob}(:,1) = [];
            end
            
            % Perform RBPF updates
            for p = 1:nParticles
                if(dCaus) % Causal update
                    robPose = data(rob).pose(:,t);
                    u = dCaus{1}; 
                    z = dCaus{2};
                    M = [alphas(1:2);alphas(3:4);alphas(5:6)]*u;
                    robOdom(:,rob,p) = SampleMotionModel(u(1),u(2),dt,robOdom(:,rob,p),M);
                    weight(p) = weight(p)*measurement_model_prob(z,robOdom(:,rob,p),map(:,:,p),SENSOR,Q);
                    % Compute the mapUpdate, which contains the log odds values to add to the map.
                    [mapUpdate, robPoseMapFrame(:,t,rob,p), laserEndPntsMapFrameInter] = inv_sensor_model(map(:,:,rob,p), z, robOdom(:,rob,p), gridSize, offset, probPrior, probOcc, probFree,SENSOR.RADIUS);
                    if (nParticles == 1)
                        laserEndPntsMapFrame{rob,p} = laserEndPntsMapFrameInter;
                    end
                    % Update the occupancy values of the map cells.
                    map(:,:,p) = map(:,:,p) + mapUpdate;
                end
                if(dAcaus) % Acausal update
                    u = dAcaus{1};
                    z = dAcaus{2};
                    % Update prediction
                    robOdom(:,rob,p) = SampleMotionModel(u(1),u(2),dt,robOdom(:,rob,p),M); % Update measurement
                    weight(p) = weight(p)*measurement_model_prob(z,robOdom(:,rob,p),map(:,:,p),SENSOR,Q);
                    [mapUpdate, robPoseMapFrame(:,t,rob,p), laserEndPntsMapFrameInter] = inv_sensor_model(map(:,:,rob,p), z, robOdom(:,rob,p), gridSize, offset, probPrior, probOcc, probFree,SENSOR.RADIUS);
                    map(:,:,p) = map(:,:,p) + mapUpdate;% Update map
                    if(dAcaus{3} ~= 0) % Check if acausal join needed
                        joinTemp = horzcat(joinTemp, dAcaus{3}); % Acausal join
                    end
                end
            end
            
        end
    end
    
    % Resample
    if (nParticles>1)
        
        weight=exp(-weight/abs(min(weight)));
        for a1=1:nRobots
            [robOdom(:,a1,:),map(:,:,a1,:),weight]=resample(robOdom(:,a1,:),map(:,:,a1,:),weight);
        end
        
        figure(2)
        colours=lines(nRobots);
        for a2=1:nParticles
            for a1=1:nRobots
                plot(robPoseMapFrame(1,t,a1,a2),robPoseMapFrame(2,t,a1,a2),'x','Color',colours(a1,:))
                hold on;
            end
        end
        hold off;
        drawnow;
    end
    
    if (nParticles==1)
        mapCombined=sum(map,3);
        % Plot current map and robot trajectory so far.
        plot_map_multi_PF(mapCombined, mapBox, robPoseMapFrame, data, laserEndPntsMapFrame, gridSize, offset, t);
        filename = sprintf('plots/gridmap_%03d.png', t);
        cropBackground(filename)
    end
    
    join = joinTemp;
end

save(sprintf('%s-BIGDATA.mat',datestr(now,30)),'map','robPoseMapFrame')
% system(sprintf('avconv -r 5 -b 0.5M -i plots/gridmap_%%03d.png %s-gridmap.mp4',datestr(now,30)))
parpool('close');


%for a1=1:size(map,3);figure(a1);imshow(ones(size(map(:,:,a1))) - log_odds_to_prob(map(:,:,a1)));axis ij equal tight;end