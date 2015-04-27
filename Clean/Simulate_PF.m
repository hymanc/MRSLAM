%Simulate_PF

%Workspace Setup
    addpath('Tools')
    addpath('COTs')

    mkdir plots
    more off
    close all
    %clear all

%Load Data
    load('../CustomMapAndModel/CustomData.mat')
    
%Simulation Parameters

    switch lower(OdometryModel)
        case 'odometrymotion'
            alphas = 0.1*[0.05 0.01 0.01 0.05].^2;%Noise properties from Table 5.6 of ProbRob
        case 'velocitymotion'
            alphas = [0.05 0.01 0.01 0.02 0.01 0.05].^2;%Noise properties from Table 5.3 of ProbRob
    end

    nParticles=30;                      %The number of particles
    robotInds=1:5%:5%:numel(data);            %Allows us to only take a subset of the robots.
    nRobots=numel(robotInds);           %Number of robots
    
    useOdometry=true;
    
    plotStuff.odom=true;
    plotStuff.odomfig=100;
    
    plotStuff.pf=true;
    plotStuff.pffig=101;
    
    plotStuff.map=true;
    plotStuff.mapfig=102;
    
    plotStuff.weight=true;
    plotStuff.weightfig=102;
    
    useParallel=false;
    if (useParallel)
        myCluster = parcluster('local');
        myCluster.NumWorkers = 4;
        saveAsProfile(myCluster,'local');
        ncores=matlabpool('size');
        if (ncores~=myCluster.NumWorkers & ncores>1)
            matlabpool('close');
            matlabpool(myCluster.NumWorkers);
        elseif (ncores~=myCluster.NumWorkers)
            matlabpool(myCluster.NumWorkers);
        end
    end

%Probabilities
    % Initial cell occupancy probability.
    probPrior = 0.50;
    % Probabilities related to the laser range finder sensor model.
    probOcc = 0.9;
    probFree = 0.35;
    logOddsPrior = prob_to_log_odds(probPrior);

%Map Properties
    gridSize=1;% Map grid size in meters. Decrease for better resolution.
    border=10;  % Set up map boundaries and initialize map.
    [pose,robXMin,robXMax,robYMin,robYMax,mapBox,offsetX,offsetY,mapSizeMeters,mapSize]=getMapParameters(useOdometry,plotStuff,data,robotInds,nRobots,border,gridSize,alphas,OdometryModel);

    % The occupancy value of each cell in the map is initialized with the prior.
    %map = logOddsPrior*ones([mapSize nRobots nParticles]);
    map = logOddsPrior*ones([mapSize nParticles]);
    %mapUpdate = zeros([mapSize nRobots nParticles]);
    mapCombined = logOddsPrior*ones(mapSize);
    disp('Map initialized. Map size:'), disp(size(map))

    % Map offset used when converting from world to map coordinates.
    offset = [offsetX; offsetY];


% Main loop for updating map cells.
% You can also take every other point when debugging to speed up the loop (t=1:2:size(poses,1))
    robOdom=squeeze(pose(:,1,:));
    robOdom=repmat(robOdom,[1 1 nParticles]);
    robPoseMapFrame=zeros([2 size(data(1).pose,2) nRobots nParticles]);
    weight=1/nParticles*ones(nParticles);
    for t=1:(size(data(1).pose,2)-1)
        t
        % Robot pose at time t.
        parfor a2=1:nParticles
            for a1=1:nRobots
                if (size(data(robotInds(a1)).pose,2)>=t)
                    % Laser scan made at time t.
                    sc=data(robotInds(a1)).r{t};
                    %weight(a1,a2)=measurement_model_prob(sc,robOdom(:,a1,a2),sum(map(:,:,:,a2),3),SENSOR,Q);
                    weight(a2)=weight(a2)*measurement_model_prob(sc,robOdom(:,a1,a2),sum(map(:,:,a2),3),SENSOR,Q,R);

                    % Compute the mapUpdate, which contains the log odds values to add to the map.
                    if (useOdometry)
                        robOdom(:,a1,a2)=Odometry(data(robotInds(a1)).u(:,t),robOdom(:,a1,a2),alphas,OdometryModel);
                        %[mapUpdate, robPoseMapFrame(:,t,a1,a2), laserEndPntsMapFrameInter] = inv_sensor_model(map(:,:,a1,a2), sc, robOdom(:,a1,a2), gridSize, offset, probPrior, probOcc, probFree,SENSOR.RADIUS);
                        [mapUpdate, robPoseMapFrame(:,t,a1,a2), laserEndPntsMapFrameInter] = inv_sensor_model(map(:,:,a2), sc, robOdom(:,a1,a2), gridSize, offset, probPrior, probOcc, probFree,SENSOR.RADIUS);
                    else
                        robPose=data(robotInds(a1)).pose(:,t);
                        %[mapUpdate, robPoseMapFrame(:,t,a1,a2), laserEndPntsMapFrameInter] = inv_sensor_model(map(:,:,a1,a2), sc, robPose, gridSize, offset, probPrior, probOcc, probFree,SENSOR.RADIUS);
                        [mapUpdate, robPoseMapFrame(:,t,a1,a2), laserEndPntsMapFrameInter] = inv_sensor_model(map(:,:,a2), sc, robPose, gridSize, offset, probPrior, probOcc, probFree,SENSOR.RADIUS);
                    end
                    if (nParticles==1)
                        laserEndPntsMapFrame{a1,a2}=laserEndPntsMapFrameInter;
                    end                
                    %map(:,:,a1,a2)=map(:,:,a1,a2)+mapUpdate;
                    map(:,:,a2)=map(:,:,a2)+mapUpdate;
                end
            end
        end
        script_PFResample;


    end

save(sprintf('%s-BIGDATA.mat',datestr(now,30)),'map','robPoseMapFrame')
    
    
    
    
    
% system(sprintf('avconv -r 5 -b 0.5M -i plots/gridmap_%%03d.png %s-gridmap.mp4',datestr(now,30)))
%matlabpool('close');

%for a1=1:size(map,3);figure(a1);imshow(ones(size(map(:,:,a1))) - log_odds_to_prob(map(:,:,a1)));axis ij equal tight;end

% if (nParticles==1)
%     mapCombined=sum(map,3);
%     % Plot current map and robot trajectory so far.
%     plot_map_multi_PF(mapCombined, mapBox, robPoseMapFrame, data, laserEndPntsMapFrame, gridSize, offset, t);
%     filename = sprintf('plots/gridmap_%03d.png', t);
%     cropBackground(filename)
% end