%Reverse Queue

%If there is anything in the reverse queue
    if (revcounters(a1)>1)
         parfor a2=1:nParticles
            sc=revqueue(a1,revcounters(a1)-1).scan;

            % Compute the mapUpdate, which contains the log odds values to add to the map.
            if (useOdometry)
                robOdomReverse(:,a1,a2)=ReverseOdometry(revqueue(a1,revcounters(a1)).u,robOdomReverse(:,a1,a2),alphas,OdometryModel);% Note the 0*alpha, we are propagating the forward dynamics.  I personally think another PF should be here.
            else
                robOdomReverse(:,a1,a2)=revqueue(a1,revcounters(a1)-1).pose;
            end
            [mapUpdate, robPoseMapFrameReverse(:,t,a1,a2), laserEndPntsMapFrameInter] = inv_sensor_model(map(:,:,a2), sc, robOdomReverse(:,a1,a2), gridSize, offset, probPrior, probOcc, probFree,SENSOR.RADIUS);
            map(:,:,a2)=map(:,:,a2)+mapUpdate;

            weight(a2)=weight(a2)*measurement_model_prob(sc,robOdomReverse(:,a1,a2),map(:,:,a2),SENSOR,Q,R,gridSize,offset);
         end
        odomRevBarDiff(:,t)=robOdomReverse(:,a1,a2)-data(robotInds(a1)).pose(:,revqueue(a1,revcounters(a1)).t-1);
        odomRevBarDiff(3,t)=normalizeTheta(odomRevBarDiff(3,t));

%                 figure(14543)
%                     for a2=1:3
%                         subplot(3,1,a2)
%                         plot(odomRevBarDiff(a2,:),'k');
%                     end
        %revqueue(a1,revcounters(a1))=struct('scan',[],'pose',[],'u',[],'t',[]);
        
        % Check for acausal joins
        enc = revqueue(a1,revcounters(a1)).encounter;
        if(size(enc,2)>1)
            % Prior encounter, perform acausal join
            oldrob=robID(robotInds==enc(1,2));
            newrob=robID(robotInds==enc(1,3));
            if(~find(joined==newrob))
                counters(newrob)=counters(newrob)-1;%t-1.
                fprintf('Robot %d was invited by Robot %d.\n',enc(1,3),enc(1,2));
                joined=[joined;newrob];
                for a2=1:counters(newrob)
                    revqueue(newrob,a2)=queue(newrob,a2);
                    queue(newrob,a2)=struct('scan',[],'pose',[],'u',[],'t',[],'encounter',[]);
                end

                revcounters(newrob)=counters(newrob);
                counters(newrob)=1;

                relpose=enc(1,4:6);
                for a2=1:nParticles
                    robOdom(:,newrob,a2)=robOdom(:,oldrob,a2)+relpose';
                    robPoseMapFrame(:,t,newrob,a2)=world_to_map_coordinates(robOdom(1:2,newrob,a2), gridSize, offset);
                    robOdomReverse(:,newrob,a2)=robOdom(:,oldrob,a2)+relpose';
                    robPoseMapFrameReverse(:,t,newrob,a2)=world_to_map_coordinates(robOdomReverse(1:2,newrob,a2), gridSize, offset);
                end 
            end
        end
        % End of acausal join
        
        revcounters(a1)=revcounters(a1)-1;
    end