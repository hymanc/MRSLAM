    

load('../CustomMapAndModel/CustomData1.mat')
% filenames={'20150427T191919-BIGDATA-OursHigh.mat','20150427T195209-BIGDATA-OursLow.mat',...
%     '20150427T201239-BIGDATA-HowardHigh.mat','20150427T203227-BIGDATA-HowardLow.mat',...
%     '20150427T203721-BIGDATA-KnownPoses.mat','20150427T205708-BIGDATA-PureOdometry.mat'};

filenames={'20150427T191919-BIGDATA-OursHigh.mat','20150427T195209-BIGDATA-OursLow.mat',...
    '20150427T201239-BIGDATA-HowardHigh.mat','20150427T203227-BIGDATA-HowardLow.mat'};



offset=[1.8669 1.3991]';

PoseErr=zeros(size(robPoseMapFrame,3),size(robPoseMapFrame,2),numel(filenames));

for a1=1:numel(filenames)
    load(filenames{a1});
    
    errvect=zeros(size(robPoseMapFrame,4),1);
    for a2=1:size(robPoseMapFrame,2)
        for a3=1:size(robPoseMapFrame,3)
            robPose=world_to_map_coordinates(data(a3).pose(1:2,a2),1,offset);
            for a4=1:size(robPoseMapFrame,4)
                if (~isnan(robPoseMapFrame(:,a2,a3,a4)))
                    errvect(a4)=norm(robPose-robPoseMapFrame(:,a2,a3,a4));
                else
                    errvect(a4)=0.01;
                end
            end
            PoseErr(a3,a2,a1)=min(errvect);
        end
    end
end


figure(1)
    plot(squeeze(sum(PoseErr(:,1:250,:),1)));
    set(gca,'FontSize',14,'FontName','Times')
    xlabel('Time [s]')
    ylabel('Minimum Sum Error [m]');
    legend({'Proposed: High','Proposed: Low','Howard: High','Howard: Low'},'Location','NW')
