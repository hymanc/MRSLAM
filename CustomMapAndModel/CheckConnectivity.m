function CheckConnectivity()

    load('CustomData.mat')
    [MAP,PIXDIM]=getTheMAP(THEIMAGE);
    
    encounterMAP=[];
    for a1=1:size(data(1).pose,2)
        for a2=1:numel(data)
            r1=customMapMeasurement(data(a2).pose(1,a1),data(a2).pose(2,a1),data(a2).pose(3,a1),MAP,PIXDIM,SENSOR);
            for a3=setdiff(1:numel(data),a2);
                MAPplusrobot=MAP;
                rx=round(data(a3).pose(1));
                ry=round(data(a3).pose(2));
                MAPplusrobot((rx-2):(rx+2),(ry-2):(ry+2))=1;
                r2=customMapMeasurement(data(a2).pose(1,a1),data(a2).pose(2,a1),data(a2).pose(3,a1),MAPplusrobot,PIXDIM,SENSOR);
                if (~(sum(abs(r2-r1))<1E-9))
                    encounterMAP=[encounterMAP;a1 a2 a3];%Time, Robot Sensing, Robot Sensed
                end
            end
        end
    end
    
    z=exp(2*pi*1i*((1:numel(data))-1)/numel(data));
    figure(102)
        plot(real(z),imag(z),'o');
        hold on;
        for a1=1:size(encounterMAP,1);
            plot(real(z(encounterMAP(a1,2:3))),imag(z(encounterMAP(a1,2:3))),'-');
        end
    
    
end