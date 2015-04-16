function [rad,pose,odom]=readAlbertB()

    filename='../Data/albertB.img.sm.log';
    fff=fopen(filename,'r');
    
    FLASER='FLASER';
    nFLASER=numel(FLASER);
    str=[];
    rad=[];
    pose=[];
    odom=[];
    c1=1;
    while (~feof(fff))
        str=fgetl(fff);
        if (numel(str)>nFLASER)
            if (strcmp(str(1:nFLASER),FLASER))
                [posenew,radnew,odomnew]=parseString(str((nFLASER+2):end));
                rad=[rad radnew];
                pose=[pose posenew];
                odom=[odom odomnew];
                c1=c1+1;
            end
        end
    end

    fclose(fff);
end


function [pose,rad,odom]=parseString(str)
    nmeas=str2double(str(1:3));
    str=str(5:end);
    
    V=textscan(str,'%f');
    pose=V{1}(181:183);
    rad=V{1}(1:180);
    odom=V{1}(184:186);
end