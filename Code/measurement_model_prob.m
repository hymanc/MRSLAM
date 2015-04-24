function w=measurement_model_prob(sc,pose,MAP,SENSOR,Q,R,gridSize,offset)

    r=rayTrace(pose(1),pose(2),pose(3),MAP>prob_to_log_odds(0.75),SENSOR);
    
    smap=size(MAP);
    
    robTrans = v2t(pose);
    robPoseMapFrame=world_to_map_coordinates(pose(1:2),gridSize,offset);
    laserEndPnts = robotlaser_as_cartesian(sc, SENSOR.RADIUS, false);
    laserEndPnts = robTrans*laserEndPnts;
    laserEndPntsMapFrame=world_to_map_coordinates(laserEndPnts(1:2,:),gridSize,offset);
    
    MAP=log_odds_to_prob(MAP)>1E-1;
    
    w0=0;
    for sc=1:columns(laserEndPntsMapFrame)
        [X,Y] = bresenham([robPoseMapFrame(1:2)';laserEndPntsMapFrame(1,sc) laserEndPntsMapFrame(2,sc)]);
        freeCells=sub2ind(smap,X,Y);
        occpCells=sub2ind(smap,laserEndPntsMapFrame(1,sc),laserEndPntsMapFrame(2,sc));
        
        FREE=1.0*(MAP(freeCells)==1);%Yes, 1 to avoid subtracting later on, gets the mismatched free cells.
        OCCP=1.0*(MAP(occpCells)==0);%Yes, 0 to avoid subtracting later on, gets the mismatched occupied cells. 
        
        w0=w0+0.1*(FREE*FREE'+OCCP*OCCP');
    end
    
    w=Q^-1*sum((r-sc).^2)+w0;
    
end

function r=rayTrace(x,y,theta,MAP,SENSOR)

    phi=linspace(SENSOR.AOS(1),SENSOR.AOS(2),SENSOR.AOSDIV);
    RR=1:SENSOR.RADIUS;
    
    PIXDIM=1;
    
    sMAP=size(MAP);
    PIX=1/PIXDIM;
    X=round(x/PIX)+1;
    Y=round(y/PIX)+1;
    
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