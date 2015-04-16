function scanAndFillCustom()

    close all;
    
    %[x,y,theta]=MYpath();
    
    figure(1)    
    NEWMAP=zeros(1000,1000);
    %NEWMAP=zeros(1139,1588);
    sNEWMAP=size(NEWMAP);
    
    STEPS=3000;
    NROBOTS=2;
    x=zeros(STEPS,NROBOTS);
    y=zeros(STEPS,NROBOTS);
    theta=zeros(STEPS,NROBOTS);
    %Test 1
     x(1,:)=500;
     y(1,:)=300;
    %Test 3
%     x(1,:)=505;
%     y(1,:)=610;
    %Test 4
%     x(1,:)=450;
%     y(1,:)=975;
    theta(1,:)=linspace(0,2*pi-2*pi/NROBOTS,NROBOTS);
    
    colours=lines(NROBOTS);
    
    c1=1;
    while (c1<STEPS)
        figure(1)
        hold off;
        for a1=1:NROBOTS
            [r,phi]=customMapMeasurement(x(c1,a1),y(c1,a1),theta(c1,a1));

            
            if (~isempty(r))
                MU=[];
                for a2=1:numel(r)
                    MU=measurementToPosition([x(c1,a1);y(c1,a1);theta(c1,a1)],[r(a2);phi(a2)]);

                    MU=round(MU);
                    if (MU(1)<=0)
                        MU(1)=1;
                    end

                    if (MU(2)<=0)
                       MU(2)=1; 
                    end

                    if (sNEWMAP(1)<=MU(1))
                        MU(1)=sNEWMAP(1);
                    end

                    if (sNEWMAP(2)<=MU(2))
                       MU(2)=sNEWMAP(2); 
                    end
                    %NEWMAP(MU(1),MU(2))=NEWMAP(MU(1),MU(2))+1;
                    NEWMAP(MU(1),MU(2))=1;
                end
            end
            
            [xnew,ynew,thetanew]=walkThisWay(x(c1,a1),y(c1,a1),theta(c1,a1),NEWMAP,c1,a1);
            x(c1+1,a1)=xnew;
            y(c1+1,a1)=ynew;
            theta(c1+1,a1)=thetanew;
        end
        
        imagesc(NEWMAP)
        colormap('gray')
        axis image;
        hold on;
        if (c1>10)
            for a1=1:NROBOTS
                plot(y((c1-10):c1,a1),x((c1-10):c1,a1),'Color',colours(a1,:))
            end
        else
            for a1=1:NROBOTS
                plot(y(1:c1,a1),x(1:c1,a1),'Color',colours(a1,:))
            end
        end
        c1=c1+1;
        drawnow;
    end
end


function [xnew,ynew,thetanew]=walkThisWay(x,y,theta,MAP,a1,signum)


    signum=(-1)^signum;
    thetanew=theta;
    c1=1;
    multiplier=1;
    if (a1<500 | a1>2000)
        multiplier=0;
    end
    
    while (true)
        dx=6;%5*rand+1;
        dtheta=multiplier*(-pi/12)+randn*pi/90;
        thetanew=thetanew+signum*dtheta;
        
        xnew=x+dx*cos(thetanew);
        ynew=y+dx*sin(thetanew);
        
        if (MAP(round(xnew),round(ynew))==0)
            break;
        else
            dtheta=-3*dtheta-2*randn*pi/90;
            thetanew=thetanew+signum*dtheta;
        end
        c1=c1+1;
        
    end

    
end


function [x,y,theta]=MYpath()

    x=[];
    y=[];
    theta=[];
    
%     %Diagonal
%     x=[x linspace(950,10,1000)];
%     y=[y linspace(950,10,1000)];
%     theta=[theta linspace(-5*pi/4,-5*pi/4,1000)];
%     
%     
%     %Right
%     x=[x linspace(10,1000,1000)];
%     y=[y linspace(10,10,1000)];
%     theta=[theta linspace(0,0,1000)];

    theta=linspace(0,10*2*pi,1000);
    x=480+linspace(0,500,1000).*cos(theta);
    y=500+linspace(0,500,1000).*sin(theta);


end



function MU=measurementToPosition(STATE,Z)

    MU=STATE(1:2)+[Z(1)*cos(STATE(3)+Z(2));Z(1)*sin(STATE(3)+Z(2))];
end