% gridmapMultiHoward.m
% Multi-Robot FastSLAM Implementation based on 
% "Multi-robot Simultaneous Localization and Mapping using Particle
% Filters"
% 20 APR 2015
% R. Choroszucha, A. Collier, C. Hyman
% EECS 568
% University of Michigan

addpath('tools')

mkdir plots
more off
close all
%clear all

% Parallel PF execution cluster
%myCluster = parcluster('local');
%myCluster.NumWorkers = 4;
%saveAsProfile(myCluster,'local');
%parpool(myCluster.NumWorkers);

% Load laser scans and robot poses.
%load('../Data/CustomData-10Robots.mat')
load('../Data/CustomData-Straighter.mat')

% Noise parameters
alphas = [0.02 0.02 0.02 0.02 0.1 0.1].^2;

% Number of Maps/Particles
nParticles=30;

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
border = 30;

%%{
%figure(1)
%pose = repmat(data(1).pose(:,2),[1 nRobots]);
T = size(data(1).pose,2)-1;
pose = zeros([3,nRobots,T]);
for rob = 1:nRobots % Build ground truth pose
    for t = 1:T+1
        pose(:,rob,t) = data(rob).pose(:,t);
    end
end

robXMin = min(min(pose(2,:,:)));
robXMax = max(max(pose(2,:,:)));%+50;
robYMin = min(min(pose(1,:,:)));
robYMax = max(max(pose(1,:,:)));%+50;

mapBox = [robXMin-border robXMax+border robYMin-border robYMax+border];
mapSizeMeters = [mapBox(2)-mapBox(1) mapBox(4)-mapBox(3)];
mapSize = ceil(mapSizeMeters./gridSize);

% Used when updating the map. Assumes that prob_to_log_odds.m
% has been implemented correctly.
logOddsPrior = prob_to_log_odds(probPrior);

% The occupancy value of each cell in the map is initialized with the prior.
map = logOddsPrior*ones([mapSize nParticles]); 
mapCombined = logOddsPrior*ones(mapSize);
disp('Map initialized. Map size:'), disp(size(map))

% Map offset used when converting from world to map coordinates.
offset = [mapBox(1);mapBox(3)];

poseMap = cell(nRobots,1);
for rob = 1:nRobots
    poseMap{rob} = world_to_map_coordinates(squeeze(pose(1:2,rob,:)), gridSize, offset);
end

%% Pre/post encounter queues
join = 1; % Joined/Post encounter list (initialize to at least one robot)
aQ = cell(nRobots,1);
cQ = cell(nRobots,1);
% For each enqueued cell (4x1):
% Param 1: actions (u)
% Param 2: measurements (z)
% Param 3: observed robots
% Param 4: observed robot poses

% Robot pose states
xRc = cell(nRobots,1); % Causal poses
xRa = cell(nRobots,1); % Acausal poses
for rob = 1:nRobots
    xRc{rob} = zeros(3,nParticles);
    xRa{rob} = zeros(3,nParticles);
end
xRc{1} = repmat(pose(:,1,1), [1 nParticles]); % Initialize pioneer particles

%robOdom = repmat(robOdom,[1 1 nParticles]);
robPoseMapFrameC = zeros([2 size(data(1).pose,2) nRobots nParticles]);
robPoseMapFrameA = zeros([2 size(data(1).pose,2) nRobots nParticles]);
weight = 1/nParticles*ones(nParticles,1); % Initial weights
% TODO: Update until all causal/non-causal data are exhausted
%% Main SLAM loop
for t=1:T%(size(data(1).pose,2)-1)
    t
    joinTemp = join; % Alias join to enforce wait for newly joined robots
    % Append queues
    for rob = 1:nRobots
        if(size(data(rob).pose,2)>=t)
            update = cell(4,1);
            update{1} = data(rob).u(:,t); %[data(rob).vact(t) ; data(rob).omegaact(t)]; % u
            update{2} = data(rob).r{t}; % z
            update{3} = 0; % rob
            update{4} = zeros(3,1);  % Delta (relative pose
            for sighting = 1:nRobots % Check for encounter
                if(sighting ~= rob)
                    rpose = pose(:,sighting,t+1)-pose(:,rob,t+1);
                    if( sqrt(rpose(1).^2 + rpose(2).^2) < 30)
                        % TODO: Check for occlusion
                        % Check for causal join
                        if(find(join==rob))
                            if(size(find(join==sighting),2)==0)
                                disp 'JOINING'
                                % Initialize pose
                                for p = 1:nParticles
                                    spose = xRc{rob}(:,p) + rpose;
                                    xRc{sighting}(:,p) = spose;
                                    xRa{sighting}(:,p) = spose;
                                end
                                joinTemp = [joinTemp, sighting];
                            end
                        end
                        update{3} = [update{3}, sighting]; % Sighting index
                        update{4} = [update{4}, rpose];    % Sighting rel. pose
                    end
                end
            end
            % Add data to causal/acausal queues
            if(find(join==rob)) % Robot post-encounter, add to causal queue
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
                if(size(dCaus)) % Causal update
                    u = dCaus{1};
                    z = dCaus{2};
                    M = [alphas(1:2);alphas(3:4);alphas(5:6)]*u;
                    xRc{rob}(:,p) = OdometryMotion(u,xRc{rob}(:,p),alphas);%SampleMotionModel(u(1),u(2),dt,xRc{rob}(:,p),M);
                    weight(p) = weight(p) * measurement_model_prob(z,xRc{rob}(:,p),map(:,:,p),SENSOR,Q);
                    % Compute the mapUpdate, which contains the log odds values to add to the map.
                    [mapUpdate, robPoseMapFrameC(:,t,rob,p), laserEndPntsMapFrameInter] = inv_sensor_model(map(:,:,p), z, xRc{rob}(:,p), gridSize, offset, probPrior, probOcc, probFree,SENSOR.RADIUS);
                    if (nParticles == 1)
                        laserEndPntsMapFrame{rob,p} = laserEndPntsMapFrameInter;
                    end
                    % Update the occupancy values of the map cells.
                    map(:,:,p) = map(:,:,p) + mapUpdate;
                end
                if(size(dAcaus)) % Acausal update
                    u = dAcaus{1}; % Reverse odometry
                    z = dAcaus{2};
                    % Update prediction
                    xRa{rob}(:,p) = OdometryMotionReverse(u,xRa{rob}(:,p),alphas);%SampleMotionModel(u(1),u(2),dt, xRa{rob}(:,p),M);
                    weight(p) = weight(p)*measurement_model_prob(z,xRa{rob}(:,p),map(:,:,p),SENSOR,Q);
                    [mapUpdate, robPoseMapFrameA(:,t,rob,p), laserEndPntsMapFrameInter] = inv_sensor_model(map(:,:,p), z, xRa{rob}(:,p), gridSize, offset, probPrior, probOcc, probFree,SENSOR.RADIUS);
                    map(:,:,p) = map(:,:,p) + mapUpdate;% Update map
                    if(dAcaus{3} ~= 0) % Check if acausal join needed
                        encounters = dAcaus{3};
                        for i = 1:length(encounters)
                            rpose = dAcaus{4}(:,i);
                            spose = xRa{rob}(:,p) + rpose;
                            xRc{encounters(i)}(:,p) = spose;
                            xRa{encounters(i)}(:,p) = spose;
                        end
                        joinTemp = horzcat(joinTemp, encounters); % Acausal join
                    end
                end
            end
            
        end
    end
    
    % Resample
    if (nParticles>1)
        %weight = exp(1+weight/abs(max(weight))); % Filter weights
        weight = weight/sum(weight); % Normalize weight
        %plot(weight,'Color',colours(p,:)); 
        %hold on;
        [xRc, xRa, map, weight] = resampleHoward(xRc, xRa, map, weight, nRobots);
        
        figure(2)
        mapCombined = mean(map,3)';
        %plot_map(mapCombined, mapBox, robPoseMapFrame, poses, laserEndPntsMapFrame, gridSize, offset, t);
        imshow(ones(size(mapCombined)) - log_odds_to_prob(mapCombined));
        hold on;
        %plot_map_Howard(mapCombined, mapBox, robPoseMapFrame, data, laserEndPntsMapFrame, gridSize, offset, t)
        
        % Plot ground truth robots
        
        colors=lines(nRobots);
        for p=1:nParticles
            for rob=1:nRobots
                % Plot 
                plot(poseMap{rob}(1,1:t+1),poseMap{rob}(2,1:t+1),'.','Color',colors(rob,:));
                plot(poseMap{rob}(1,t+1),poseMap{rob}(2,t+1),'o','Color',colors(rob,:));
                plot(robPoseMapFrameC(1,t,rob,p),robPoseMapFrameC(2,t,rob,p),'+','Color',colors(rob,:)) % Plot PF estimates
                plot(robPoseMapFrameA(1,t,rob,p),robPoseMapFrameA(2,t,rob,p),'X','Color',colors(rob,:))
                hold on;
            end
        end
        hold off;
        drawnow;
    filename = sprintf('plots/howard_%03d.png', t);
    print(gcf,filename, '-dpng');
        cropBackground(filename)
    end
    
    if (nParticles==1)
        mapCombined = sum(map,3);
        mapCombined = mapCombined';
        % Plot current map and robot trajectory so far.
        plot_map_multi_PF(mapCombined, mapBox, robPoseMapFrame, data, laserEndPntsMapFrame, gridSize, offset, t);
        filename = sprintf('plots/gridmap_%03d.png', t);
        cropBackground(filename)
    end
    
    join = joinTemp;
end

%save(sprintf('%s-BIGDATA.mat',datestr(now,30)),'map','robPoseMapFrame')
% system(sprintf('avconv -r 5 -b 0.5M -i plots/gridmap_%%03d.png %s-gridmap.mp4',datestr(now,30)))
%parpool('close');
