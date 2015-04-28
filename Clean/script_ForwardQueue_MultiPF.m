%Forward Queue

    if (sum(a1==joined) & counters(a1)>1 & t<=maxT)                
        %Update Filter
        %robOdomOld(:,a1,:)=robOdom(:,a1,:);
        parfor a2=1:nParticles
            % Laser scan made at time t.
            sc=queue(a1,counters(a1)-1).scan;

            % Compute the mapUpdate, which contains the log odds values to add to the map.
            if (useOdometry)
                robOdom(:,a1,a2)=Odometry(queue(a1,counters(a1)-1).u,robOdom(:,a1,a2),1/4*alphas,OdometryModel);
            else
                robOdom(:,a1,a2)=data(robotInds(a1)).pose(:,t);                     
            end
            [mapUpdate, robPoseMapFrame(:,t,a1,a2), laserEndPntsMapFrameInter] = inv_sensor_model(map(:,:,a2), sc, robOdom(:,a1,a2), gridSize, offset, probPrior, probOcc, probFree,SENSOR.RADIUS*2);
            map(:,:,a2)=map(:,:,a2)+mapUpdate;
            weight(a2,a1)=weight(a2,a1)*measurement_model_prob(sc,robOdom(:,a1,a2),map(:,:,a2),SENSOR,Q,R,gridSize,offset);
        end
        counters(a1)=counters(a1)-1;
            
            
         %Moved forward, saw another robot, added it to joined.
        if (~isempty(mapEncounters))
            if (mapEncounters(1,1)==t)
                oldrob=robID(robotInds==mapEncounters(1,2));
                newrob=robID(robotInds==mapEncounters(1,3));
                
                counters(newrob)=counters(newrob)-1;%t-1.
                fprintf('Robot %d was invited by Robot %d.\n',mapEncounters(1,3),mapEncounters(1,2));
                joined=[joined;newrob];
                for a2=1:counters(newrob)
                    revqueue(newrob,a2)=queue(newrob,a2);
                    queue(newrob,a2)=struct('scan',[],'pose',[],'u',[],'t',[],'encounter',[]);
                end

                revcounters(newrob)=counters(newrob);
                counters(newrob)=1;

                relpose=mapEncounters(1,4:6);
                for a2=1:nParticles
                    robOdom(:,newrob,a2)=robOdom(:,oldrob,a2)+relpose';
                    robOdom(:,newrob,a2)-data(newrob).pose(:,t)
                    robPoseMapFrame(:,t,newrob,a2)=world_to_map_coordinates(robOdom(1:2,newrob,a2), gridSize, offset);
                    robOdomReverse(:,newrob,a2)=robOdom(:,oldrob,a2)+relpose';
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