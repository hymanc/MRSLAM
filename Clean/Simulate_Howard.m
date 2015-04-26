%Simulate_PF

%Workspace Setup
    addpath('Tools')
    addpath('COTs')
    
    mkdir plots
    more off
    %close all
    %clear all

%Load Data
    load('../CustomMapAndModel/CustomData1.mat')
    
%Simulation Parameters

    switch lower(OdometryModel)
        case 'odometrymotion'
            alphas =1/2*[0.05 0.01 5*[0.01 0.05]].^2;%Noise properties from Table 5.6 of ProbRob
        case 'velocitymotion'
            alphas = [0.05 0.01 0.01 0.02 0.01 0.05].^2;%Noise properties from Table 5.3 of ProbRob
    end

    nParticles=30;                      %The number of particles
    robotInds=[1 3 5]% 2 3 4 5];%:numel(data);            %Allows us to only take a subset of the robots.
    nRobots=numel(robotInds);           %Number of robots
    robID=1:nRobots;
    
    useOdometry=true;
    
    plotStuff.fontsize=14;
    plotStuff.fontname='Times';
    
    plotStuff.odom=true;
    plotStuff.odomfig=100;
    
    plotStuff.pf=true;
    plotStuff.pffig=101;
    
    plotStuff.map=true;
    plotStuff.mapfig=102;
    
    plotStuff.weight=true;
    plotStuff.weightfig=102;
    
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
    border=SENSOR.RADIUS;  % Set up map boundaries and initialize map.
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
    
    DONE=false;
    counters=ones(nRobots,1);
    revcounters=counters;
    maxT=numel(data(1).r);
    
    joined=[1];
    unjoined=setdiff(robotInds,joined);
    mapEncounters=parseEncounters(mapEncounters,joined,robotInds);
    t=2;
    meanMAP=sum(map,3);
    
    for a1=1:nRobots
        for a2=1:nParticles
            robPoseMapFrame(:,1,a1,a2)=world_to_map_coordinates(data(robotInds(a1)).pose(1:2,1),gridSize,offset);
        end
    end
    
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
            
            
            %If there is anything in the reverse queue
            if (revcounters(a1)>1)
               parfor a2=1:nParticles
                    sc=revqueue(a1,revcounters(a1)).scan;

                    % Compute the mapUpdate, which contains the log odds values to add to the map.
                    if (useOdometry)
                        %robOdomReverse(:,a1,a2)=ReverseOdometry(revqueue(a1,revcounters(a1)).u,robOdomReverse(:,a1,a2),0*alphas,OdometryModel);% Note the 0*alpha, we are propagating the forward dynamics.  I personally think another PF should be here.
                        
                        robOdomReverse(:,a1,a2)=ReverseOdometry(data(robotInds(a1)).u(:,revqueue(a1,revcounters(a1)).t),robOdomReverse(:,a1,a2),alphas,OdometryModel);% We need the correct u for the reverse dynamics.
                    else
                        robOdomReverse(:,a1,a2)=revqueue(a1,revcounters(a1)).pose;
                    end
                    [mapUpdate, robPoseMapFrameReverse(:,t,a1,a2), laserEndPntsMapFrameInter] = inv_sensor_model(map(:,:,a2), sc, robOdomReverse(:,a1,a2), gridSize, offset, probPrior, probOcc, probFree,SENSOR.RADIUS);
                    map(:,:,a2)=map(:,:,a2)+mapUpdate;

                   weight(a2)=weight(a2)*measurement_model_prob(sc,robOdomReverse(:,a1,a2),meanMAP+mapUpdate,SENSOR,Q,R,gridSize,offset);
                   %weight(a2)=weight(a2)*measurement_model_prob(sc,robOdomReverse(:,a1,a2),map(:,:,a2),SENSOR,Q,R,gridSize,offset);
               end
               %revqueue(a1,revcounters(a1))=struct('scan',[],'pose',[],'u',[],'t',[]);
               revcounters(a1)=revcounters(a1)-1;
            end
            
            
            if (sum(a1==joined) & counters(a1)>1 & t<=maxT)
                
                
                %Update Filter
                parfor a2=1:nParticles
                    % Laser scan made at time t.
                    sc=queue(a1,counters(a1)-1).scan;
                    
                    % Compute the mapUpdate, which contains the log odds values to add to the map.
                    if (useOdometry)
                        robOdom(:,a1,a2)=Odometry(queue(a1,counters(a1)-1).u,robOdom(:,a1,a2),alphas,OdometryModel);
                    else
                        robOdom(:,a1,a2)=data(robotInds(a1)).pose(:,t);                     
                    end
                    [mapUpdate, robPoseMapFrame(:,t,a1,a2), laserEndPntsMapFrameInter] = inv_sensor_model(map(:,:,a2), sc, robOdom(:,a1,a2), gridSize, offset, probPrior, probOcc, probFree,SENSOR.RADIUS);
                    map(:,:,a2)=map(:,:,a2)+mapUpdate;
                    
                    
%                     figure(1)
%                     imagesc(1-log_odds_to_prob(map(:,:,a2)));colormap gray;axis image;axis off;
                    
                    weight(a2)=weight(a2)*measurement_model_prob(sc,robOdom(:,a1,a2),meanMAP+mapUpdate,SENSOR,Q,R,gridSize,offset);
                    %weight(a2)=weight(a2)*measurement_model_prob(sc,robOdom(:,a1,a2),map(:,:,a2),SENSOR,Q,R,gridSize,offset);
                end
                
                counters(a1)=counters(a1)-1;

                %Moved forward, saw another robot.
                
                %Check for causal joins
                if (~isempty(mapEncounters))
                    if (mapEncounters(1,1)==t)
                        oldrob=robID(robotInds==mapEncounters(1,2));
                        newrob=robID(robotInds==mapEncounters(1,3));
                        
                        counters(newrob)=counters(newrob)-1;%t-1.
                        fprintf('Robot %d was invited by Robot %d.\n',mapEncounters(1,3),mapEncounters(1,2));
                        joined=[joined;newrob];
                        for a2=1:counters(newrob)
                            revqueue(newrob,a2)=queue(newrob,a2);
                            queue(newrob,a2)=struct('scan',[],'pose',[],'u',[],'t',[]);
                        end
                        
                        revcounters(newrob)=counters(newrob);
                        counters(newrob)=1;
                        
                        relpose=mapEncounters(1,4:6);
                        for a2=1:nParticles
                            robOdom(:,newrob,a2)=robOdom(:,oldrob,a2)+relpose';
                            %robOdom(:,newrob,a2)=data(robotInds(newrob)).pose(:,t);
                            robPoseMapFrame(:,t,newrob,a2)=world_to_map_coordinates(robOdom(1:2,newrob,a2), gridSize, offset);
                            robOdomReverse(:,newrob,a2)=robOdom(:,oldrob,a2)+relpose';
                            %robOdomReverse(:,newrob,a2)=data(robotInds(newrob)).pose(:,t);
                            robPoseMapFrameReverse(:,t,newrob,a2)=world_to_map_coordinates(robOdomReverse(1:2,newrob,a2), gridSize, offset);
                        end
                                                
                        if (size(mapEncounters,1)>1)
                            mapEncounters=mapEncounters(2:end,1:end);
                        else
                            mapEncounters=[];
                        end    
                    end
                end
                
                
            end
            
            
            
            
        end
        
        
        script_PFResample;
        
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