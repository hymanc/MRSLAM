function [rad,pose,odom,t]=readAlbertB()

    filename='../Data/albertB.img.sm.log';
    fff=fopen(filename,'r');
    
    ODOM='ODOM';
    nODOM=numel(ODOM);
    FLASER='FLASER';
    nFLASER=numel(FLASER);
    str=[];
    rad=[];
    pose=[];
    odom=[];
    t=[];
    
    odomodom=[];
    inputsodom=[];
    todom=[];
    
    c1=1;
    while (~feof(fff))
        str=fgetl(fff);
        if (numel(str)>nFLASER)
            if (strcmp(str(1:nFLASER),FLASER))
                [posenew,radnew,odomnew,tnew]=parseFLaserString(str((nFLASER+2):end));
                rad=[rad radnew];
                pose=[pose posenew];
                odom=[odom odomnew];
                t=[t tnew];
                c1=c1+1;
            end
        end
        
        if (numel(str)>nODOM)
            if (strcmp(str(1:nODOM),ODOM))
                [odomnew,inputs,tnew]=parseOdomString(str((nODOM+2):end));
                odomodom=[odomodom odomnew];
                inputsodom=[inputsodom inputs];
                todom=[todom tnew];
                c1=c1+1;
            end
        end
    end

    fclose(fff);
    save('albert.mat','odom','pose','rad','t','odomodom','inputsodom','todom')
end


function [pose,rad,odom,t]=parseFLaserString(str)
    nmeas=str2double(str(1:3));
    str=str(4:end);
    
    numsf=repmat('%f ',[1 nmeas+7]);
    
    V=textscan(str,[numsf '%s %f']);
    c=[1 1 1];
    rad=zeros(nmeas,1);
    pose=zeros(3,1);
    odom=zeros(3,1);
    
    for a1=1:numel(V)
        if (1<=a1 & a1<=180)
            rad(c(1))=V{a1};
            c(1)=c(1)+1;
        end
        if (181<=a1 & a1<=183)
            pose(c(2))=V{a1};
            c(2)=c(2)+1;
        end
        if (184<=a1 & a1<=186)
            odom(c(3))=V{a1};
            c(3)=c(3)+1;
        end
    end
    t=V{end};
    
end


function [odom,inputs,t]=parseOdomString(str)
    
    numsf=repmat('%f ',[1 7]);
    V=textscan(str,[numsf '%s %f']);
    odom=zeros(3,1);
    inputs=zeros(3,1);
    c=[1 1];
    for a1=1:numel(V)
        if (1<=a1 & a1<=3)
            odom(c(1))=V{a1};
            c(1)=c(1)+1;
        end
        if (4<=a1 & a1<=6)
            inputs(c(2))=V{a1};
            c(2)=c(2)+1;
        end
    end
    t=V{end};
end
