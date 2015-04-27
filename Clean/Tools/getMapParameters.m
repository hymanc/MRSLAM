function [pose,robXMin,robXMax,robYMin,robYMax,mapBox,offsetX,offsetY,mapSizeMeters,mapSize]=getMapParameters(useOdometry,plotStuff,data,robotInds,nRobots,border,gridSize,alphas,OdometryModel)

    if (useOdometry)
        
        for a1=1:nRobots
            pose(:,1,a1)=data(robotInds(a1)).pose(:,1);
            for a2=2:(size(data(1).pose,2)-1)
                pose(:,a2,a1)=Odometry(data(robotInds(a1)).u(:,a2-1),pose(:,a2-1,a1),0*alphas,OdometryModel);
            end
        end
    else
        for a1=1:nRobots
            pose(:,:,a1)=data(robotInds(a1)).pose;
        end
    end

    robXMin = min(min(pose(1,:,:)));
    robXMax = max(max(pose(1,:,:)));
    robYMin = min(min(pose(2,:,:)));
    robYMax = max(max(pose(2,:,:)));

    mapBox = [robXMin-border robXMax+border robYMin-border robYMax+border];
    offsetX = mapBox(1);
    offsetY = mapBox(3);
    mapSizeMeters = [mapBox(2)-offsetX mapBox(4)-offsetY];
    mapSize = ceil([mapSizeMeters/gridSize]);

    if (plotStuff.odom)
        colour=lines(nRobots);
        figure(plotStuff.odomfig)
        imagesc([1 1;1 1]);
        hold on;
        for a1=1:nRobots
            plot(pose(2,:,a1),pose(1,:,a1),'Color',colour(a1,:));
            
        end
        axis image;
        hold off;
    end
    
end