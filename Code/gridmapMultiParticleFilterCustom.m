addpath('tools')

mkdir plots
more off
close all
%clear all

% myCluster = parcluster('local');
% myCluster.NumWorkers = 4;
% saveAsProfile(myCluster,'local');
% matlabpool(myCluster.NumWorkers);


% Load laser scans and robot poses.
load('../Data/CustomData-10Robots.mat')
alphas = [0.05 0.01 0.01 0.02 0.01 0.05].^2;%Noise properties from Table 5.3 of ProbRob

<<<<<<< HEAD
alphas = [0.05 0.001 0.005 0.01 0.01 0.01].^2;

nParticles=30;
nRobots=1;
=======
nParticles=30;          %The number of particles
nRobots=10;             %Number of robots
>>>>>>> 3c798f3b5417a91b231be060b84dc2d3556dbc84

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
%Figure out the expected size of the grid.
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

robXMin = min(min(pose(1,:,:)));
robXMax = max(max(pose(1,:,:)))+50;
robYMin = min(min(pose(2,:,:)));
robYMax = max(max(pose(2,:,:)))+50;

mapBox = [robXMin-border robXMax+border robYMin-border robYMax+border];
offsetX = mapBox(1);
offsetY = mapBox(3);
mapSizeMeters = [mapBox(2)-offsetX mapBox(4)-offsetY];
mapSize = ceil([mapSizeMeters/gridSize]);

% Used when updating the map. Assumes that prob_to_log_odds.m
% has been implemented correctly.
logOddsPrior = prob_to_log_odds(probPrior);

% The occupancy value of each cell in the map is initialized with the prior.
map = logOddsPrior*ones([mapSize nRobots nParticles]);
mapCombined = logOddsPrior*ones(mapSize);
disp('Map initialized. Map size:'), disp(size(map))

% Map offset used when converting from world to map coordinates.
offset = [offsetX; offsetY];


% Main loop for updating map cells.
% You can also take every other point when debugging to speed up the loop (t=1:2:size(poses,1))
robOdom=robPose;
robOdom=repmat(robOdom,[1 1 nParticles]);
robPoseMapFrame=zeros([2 size(data(1).pose,2) nRobots nParticles]);
weight=1/nParticles*ones(nRobots,nParticles);
for t=1:(size(data(1).pose,2)-1)
    t
    % Robot pose at time t.
    for a2=1:nParticles
        for a1=1:nRobots
            if (size(data(a1).pose,2)>=t)
                robPose=data(a1).pose(:,t);
                M=[alphas(1:2);alphas(3:4);alphas(5:6)]*[data(a1).v(t);data(a1).omega(t)];
                robOdom(:,a1,a2)=SampleMotionModel(data(a1).v(t),data(a1).omega(t),dt,robOdom(:,a1,a2),M);
                % Laser scan made at time t.
                sc=data(a1).r{t};
                weight(a1,a2)=measurement_model_prob(sc,robOdom(:,a1,a2),map(:,:,a1,a2),SENSOR,Q);
                
                % Compute the mapUpdate, which contains the log odds values to add to the map.
                %[mapUpdate, robPoseMapFrame(:,t,a1,a2), laserEndPntsMapFrameInter] = inv_sensor_model(map(:,:,a1,a2), sc, robPose, gridSize, offset, probPrior, probOcc, probFree,SENSOR.RADIUS);
                [mapUpdate, robPoseMapFrame(:,t,a1,a2), laserEndPntsMapFrameInter] = inv_sensor_model(map(:,:,a1,a2), sc, robOdom(:,a1,a2), gridSize, offset, probPrior, probOcc, probFree,SENSOR.RADIUS);
                if (nParticles==1)
                    laserEndPntsMapFrame{a1,a2}=laserEndPntsMapFrameInter;
                end                
                %map(MODIFIED+mapSize(1)*mapSize(2)*(a1-1)+mapSize(1)*mapSize(2)*nRobots*(a2-1))=map(MODIFIED+nRobots*(a1-1)+nRobots*nParticles*(a2-1))+mapUpdate(MODIFIED);
                map(:,:,a1,a2)=map(:,:,a1,a2)+mapUpdate;
                %map(:,:,a1)=map(:,:,a1)+mapUpdate;

            end
        end
    end
    
    if (nParticles>1)
        
        colours=lines(nRobots);
        figure(1);
        for a1=1:nRobots
            weight(a1,:)=exp(1+weight(a1,:)/abs(min(weight(a1,:))));
            weight(a1,:)=weight(a1,:)/sum(weight(a1,:));
            plot(weight(a1,:),'Color',colours(a1,:));
            hold on;
            [robOdom(:,a1,:),map(:,:,a1,:),weight(a1,:)]=resample(robOdom(:,a1,:),map(:,:,a1,:),weight(a1,:));
        end
        hold off;
        
        figure(2)
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
    
    
end

save(sprintf('%s-BIGDATA.mat',datestr(now,30)),'map','robPoseMapFrame')
% system(sprintf('avconv -r 5 -b 0.5M -i plots/gridmap_%%03d.png %s-gridmap.mp4',datestr(now,30)))
% matlabpool('close');




%for a1=1:size(map,3);figure(a1);imshow(ones(size(map(:,:,a1))) - log_odds_to_prob(map(:,:,a1)));axis ij equal tight;end