function r=customMapMeasurement(x,y,theta,MAP,PIXDIM,SENSOR)


% 
%     persistent MAP;             %Bitmap
%     persistent PIXDIM;          %Dimension of pixel
    
    phi=linspace(SENSOR.AOS(1),SENSOR.AOS(2),SENSOR.AOSDIV);
    RR=1:SENSOR.RADIUS;
    
%     if (isempty(MAP))
%         [MAP,PIXDIM]=getTheMAP(THEIMAGE);
%         %MAP=MAP';
%     end
    
    sMAP=size(MAP);
    PIX=1/PIXDIM;
    X=round(x/PIX)+1;
    Y=round(y/PIX)+1;
    
    
%     figure(2)
%         imagesc(MAP)
%         hold on;
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
        
%         plot(y+RR*sin(phi(a1)+theta),x+RR*cos(phi(a1)+theta),'y')
%         %plot(Yray,Xray,'b.')
%         plot(yhit,xhit,'ks')
        
        if isempty(xhit)
            r(a1)=SENSOR.RADIUS;
        else
            r(a1)=sqrt((x-xhit)^2+(y-yhit)^2);
        end
    end
%      hold off;

end