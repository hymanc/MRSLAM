function [pose,MAP,robPose,weight] = resampleSingle(pose,MAP,robPose,weight)

    numSamples=numel(weight);
    Xbart=pose;
	MAPbart=MAP;
    rPbart=robPose;
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
		MAP(:,:,a1)=MAPbart(:,:,c1);
        robPose(:,:,:,a1)=rPbart(:,:,:,c1);
        %newsam=[newsam c1];
        weight(a1)=1/numSamples;
    end
    weight=weight/sum(weight);
end
