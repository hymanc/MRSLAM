function w=measurement_model_prob(scan,pose,MAP,SENSOR,Q,R,gridSize,offset)

    logica=MAP>prob_to_log_odds(0.5);

    r=rayTrace(pose(1)-offset(1),pose(2)-offset(2),pose(3),logica,SENSOR);
    %r=rayTrace(pose(1),pose(2),pose(3),logica,SENSOR);
    
    smap=size(MAP);
    
    robTrans = v2t(pose);
    robPoseMapFrame=world_to_map_coordinates(pose(1:2),gridSize,offset);
    laserEndPnts = robotlaser_as_cartesian(scan, SENSOR.RADIUS, false);
    laserEndPnts = robTrans*laserEndPnts;
    laserEndPntsMapFrame=world_to_map_coordinates(laserEndPnts(1:2,:),gridSize,offset);
    
    w0=0;
    for sc=1:columns(laserEndPntsMapFrame)
        [X,Y] = bresenham([robPoseMapFrame(1:2)';laserEndPntsMapFrame(1,sc) laserEndPntsMapFrame(2,sc)]);
        try
        freeCells=sub2ind(smap,X,Y);
        occpCells=sub2ind(smap,laserEndPntsMapFrame(1,sc),laserEndPntsMapFrame(2,sc));
        
        FREE=1.0*(logica(freeCells)==1);%Yes, 1 to avoid subtracting later on, gets the mismatched free cells.
        OCCP=1.0*(logica(occpCells)==0);%Yes, 0 to avoid subtracting later on, gets the mismatched occupied cells. 
        
        w0=w0+0.1*(FREE*FREE'+OCCP*OCCP');
        catch hell
            
        end
    end
    
    
    w1=Q^-1*sum((r-scan).^2);
    w=w1+10^(round(log10(w1)))*w0;
    w=1/w;%Large error is bad, so we want it weighted small.
end

function r=rayTrace(x,y,theta,MAP,SENSOR)

    phi=linspace(SENSOR.AOS(1),SENSOR.AOS(2),SENSOR.AOSDIV);
    RR=1:SENSOR.RADIUS;
    
    PIXDIM=1;
    
    sMAP=size(MAP);
    PIX=1/PIXDIM;
    X=round(x/PIX)+1;
    Y=round(y/PIX)+1;

    %figure(56);imagesc(1-log_odds_to_prob(1.0*MAP));axis image;axis off;hold on;plot(Y,X,'ro');hold off;
    
    vect=1:numel(RR);
    theones=ones(1,numel(RR));
    r=zeros(SENSOR.AOSDIV,1);
    for a1=1:SENSOR.AOSDIV
        
        Xray=round((x+RR*cos(phi(a1)+theta))/PIX);
        Yray=round((y+RR*sin(phi(a1)+theta))/PIX);
        
        Xray=max([Xray;theones],[],1);
        Yray=max([Yray;theones],[],1);
    
        Xray=min([Xray;(sMAP(1)-1)*theones],[],1);
        Yray=min([Yray;(sMAP(2)-1)*theones],[],1);
        
        ind=sub2ind(sMAP,Xray,Yray);
        xhit=Xray(min(vect(MAP(ind))));
        yhit=Yray(min(vect(MAP(ind)))); 
    
        
        if isempty(xhit)
            r(a1)=SENSOR.RADIUS;
        else
            r(a1)=sqrt((x-xhit)^2+(y-yhit)^2);
        end
    end


end