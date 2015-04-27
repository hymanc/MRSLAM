%Simulate_PF

%Workspace Setup
    addpath('Tools')
    addpath('COTs')
    
    mkdir plots
    more off
    close all
%    clear all

%Load Data
    load('../CustomMapAndModel/CustomData1.mat')
    
%Simulation Parameters

    switch lower(OdometryModel)
        case 'odometrymotion'
            alphas =[1*[0.05 0.05] 10*[0.01 0.01]].^2;%Noise properties from Table 5.6 of ProbRob
        case 'velocitymotion'
            alphas = [0.05 0.01 0.01 0.02 0.01 0.05].^2;%Noise properties from Table 5.3 of ProbRob
    end

    nParticles=30;                      %The number of particles
    robotInds=[1 2 3 4 5]% 2 3 4 5];%:numel(data);            %Allows us to only take a subset of the robots.
    nRobots=numel(robotInds);           %Number of robots
    robID=1:nRobots;
    
    useOdometry=true;
    
    colours=lines(nRobots);
    
    plotStuff.fontsize=14;
    plotStuff.fontname='Times';
    
    plotStuff.odom=false;
    plotStuff.odomfig=100;
    plotStuff.odomvisible='on';
    
    plotStuff.pf=true;
    plotStuff.pffig=101;
    plotStuff.pfvisible='on';
    
    plotStuff.map=true;
    plotStuff.mapfig=103;
    plotStuff.mapvisible='off';
    
    plotStuff.weight=true;
    plotStuff.weightfig=102;
    plotStuff.weightvisible='on';
    
    useParallel=true;
    if (useParallel)
        myCluster = parcluster('local');
        myCluster.NumWorkers = 7;
        saveAsProfile(myCluster,'local');
        ncores=matlabpool('size');
        if (ncores~=myCluster.NumWorkers & ncores>1)
            matlabpool('close');
            matlabpool(myCluster.NumWorkers);
        elseif (ncores~=myCluster.NumWorkers)
            matlabpool(myCluster.NumWorkers);
        end
    else 
        if (matlabpool('size')>1)
            matlabpool('close');
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
    border=10;SENSOR.RADIUS;  % Set up map boundaries and initialize map.
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
    %robOdom=robOdom-repmat([offset;-pi],[1 nRobots]);
    robOdom=repmat(robOdom,[1 1 nParticles]);
    robOdomReverse=0*robOdom;
    
    robPoseMapFrame=NaN([2 size(data(1).pose,2) nRobots nParticles]);
    robPoseMapFrameReverse=NaN([2 size(data(1).pose,2) nRobots nParticles]);
    weight=1/nParticles*ones(nParticles,nRobots);
    weightReverse=1/nParticles*ones(nParticles,nRobots);
    
    
    DONE=false;
    counters=ones(nRobots,1);
    revcounters=counters;
    maxT=numel(data(1).r);
    
    joined=[1];
    unjoined=setdiff(robotInds,joined);
    mapEncounters=parseEncounters(mapEncounters,joined,robotInds);
    
    t=1;
    for a1=1:nRobots
        queue(a1,counters(a1)).scan=data(robotInds(a1)).r{t};%1 is scan data
        queue(a1,counters(a1)).pose=data(robotInds(a1)).pose(:,t);
        queue(a1,counters(a1)).u=zeros(size(data(robotInds(a1)).u(:,t)));
        queue(a1,counters(a1)).t=t;
        counters(a1)=counters(a1)+1;
    end
    
    t=2;
    while (~DONE)
        t

        %Appending data to queue.
        if (t<=maxT)
            for a1=1:nRobots
                queue(a1,counters(a1)).scan=data(robotInds(a1)).r{t};%1 is scan data
                queue(a1,counters(a1)).pose=data(robotInds(a1)).pose(:,t);
                queue(a1,counters(a1)).u=data(robotInds(a1)).u(:,t-1);
                queue(a1,counters(a1)).t=t;
                counters(a1)=counters(a1)+1;
            end
        end
        
        for a1=1:nRobots
            script_ForwardQueue_MultiPF;
            script_ReverseQueue_MultiPF;
        end
        
        
        script_PFResample_MultiPF;
        
        
        %Check if all the data has been integrated.
        if (t>=maxT & sum(revcounters==1)==nRobots)
            DONE=true;
        end
        
        t=t+1;
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