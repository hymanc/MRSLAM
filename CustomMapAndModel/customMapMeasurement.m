function [r,phi]=customMapMeasurement(x,y,theta)



    persistent MAP;             %Bitmap
    persistent PIXDIM;          %Dimension of pixel
    
    SENSOR.RADIUS=20;           %Limit of the sensor
    SENSOR.AOS=[-60 60]*pi/180; %Sensor angle of sensitivity
    SENSOR.AOSDIV=180;          %Division of AOS, important for ray tracing
    
    if (isempty(MAP))
        THEIMAGE='Test3.png';
        switch THEIMAGE
            case 'Test1.png'
                A=imread(THEIMAGE);
                MAP=A(:,:,1)==0 & A(:,:,2)==0 & A(:,:,3)==0;
                PIXDIM=1;%1 meter/1 Pixel
            case 'Test2.bmp'
                A=imread(THEIMAGE);
                MAP=A(:,:,1)==0 & A(:,:,2)==0 & A(:,:,3)==0;
                PIXDIM=1;%1 meter/1 Pixel
                
            case 'Test3.png'
                A=imread(THEIMAGE);
                MAP=A(:,:,1)~=255 & A(:,:,2)~=255 & A(:,:,3)~=255;
                PIXDIM=1;%1 meter/1 Pixel
            case 'Test4.png'
                A=imread(THEIMAGE);
                MAP=A(:,:,1)~=255 & A(:,:,2)~=255 & A(:,:,3)~=255;
                PIXDIM=1;%1 meter/1 Pixel
            case 'paw-to-paw-maze-June.jpg'
                A=imread(THEIMAGE);
                MAP=A(:,:,1)<64;
                PIXDIM=1;%1 meter/1 Pixel
            otherwise

        end
        %MAP=MAP';
        
    end
    
    sMAP=size(MAP);
    
    PIXELRADIUS=SENSOR.RADIUS/PIXDIM;
    
    X=round(x/PIXDIM)+1;
    Y=round(y/PIXDIM)+1;
    
    RXmin=max([X-PIXELRADIUS 1]);
    RYmin=max([Y-PIXELRADIUS 1]);
    
    RXmax=min([X+PIXELRADIUS sMAP(1)]);
    RYmax=min([Y+1+PIXELRADIUS sMAP(2)]);
    
    SUBSET=MAP(RXmin:RXmax,RYmin:RYmax);
    
    XX=RXmin:RXmax;
    YY=RYmin:RYmax;
    
    [XX,YY]=ndgrid(XX,YY);
    
    ANGLE=mod(atan2((YY-Y),(XX-X))-theta,2*pi);
    Ainds=ANGLE>pi;
    ANGLE(Ainds)=ANGLE(Ainds)-2*pi;
    R=((YY-Y).^2+(XX-X).^2);
    indsR=R<=PIXELRADIUS^2;
    indsPhi=SENSOR.AOS(1)<=ANGLE & ANGLE<=SENSOR.AOS(2);
    
    inds=(SUBSET & indsR & indsPhi);
    Xmeas=XX(inds);
    Ymeas=YY(inds);
    
    r=sqrt(R(inds));
    phi=ANGLE(inds);
    
%      figure(1)
%          surf(MAP*1.0,'LineStyle','None')
%          colormap('gray')
% 
%     figure(3)
%         imagesc(SUBSET)
%         hold on;
%         plot3(Xmeas,Ymeas,ones(sum(inds(:)),1),'r.')
%         hold off;
%         colormap('gray')
%      figure(4)
%          imagesc(indsR & indsPhi)
end