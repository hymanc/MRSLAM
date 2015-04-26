function [pose,poseRev,MAP,robPose,robPoseRev,weight] = resample(pose,poseRev,MAP,robPose,robPoseRev,weight)

    numSamples=numel(weight);
    Xbart=pose;
    Xrevbart=poseRev;
	MAPbart=MAP;
    rPbart=robPose;
    rPRbart=robPoseRev;
    W=cumsum(weight);
    s0=1/numSamples*rand;
    s=mod(linspace(s0,1+s0,numSamples),1);
    %newsam=[];
    for a1=1:numSamples
        c1=1;
        while (s(a1)>W(c1))
            c1=c1+1;
        end
        pose(:,:,a1)=Xbart(:,:,c1);
        poseRev(:,:,a1)=Xrevbart(:,:,c1);
		MAP(:,:,a1)=MAPbart(:,:,c1);
        robPose(:,:,:,a1)=rPbart(:,:,:,c1);
        robPoseRev(:,:,:,a1)=rPRbart(:,:,:,c1);
        %newsam=[newsam c1];
        weight(a1)=1/numSamples;
    end
    weight=weight/sum(weight);
end
