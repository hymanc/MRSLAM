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

            weightReverse(a2,a1)=weightReverse(a2,a1)*measurement_model_prob(sc,robOdomReverse(:,a1,a2),map(:,:,a2),SENSOR,Q,R,gridSize,offset);
         end
        odomRevBarDiff(:,t)=robOdomReverse(:,a1,a2)-data(robotInds(a1)).pose(:,revqueue(a1,revcounters(a1)).t-1);
        odomRevBarDiff(3,t)=normalizeTheta(odomRevBarDiff(3,t));

%                 figure(14543)
%                     for a2=1:3
%                         subplot(3,1,a2)
%                         plot(odomRevBarDiff(a2,:),'k');
%                     end
        %revqueue(a1,revcounters(a1))=struct('scan',[],'pose',[],'u',[],'t',[]);
        revcounters(a1)=revcounters(a1)-1;
    end