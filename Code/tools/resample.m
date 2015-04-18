function [pose,MAP,weight] = resample(pose,MAP,weight)

    numSamples=numel(weight);
    Xbart=pose;
	MAPbart=MAP;
    W=cumsum(weight);
    s0=1/numSamples*rand;
    s=mod(linspace(s0,1+s0,numSamples),1);
    %newsam=[];
    for a1=1:numSamples
        c1=1;
        while (s(a1)>W(c1))
            c1=c1+1;
        end
        pose(:,1,a1)=Xbart(1:3,c1);
		MAP(:,:,1,a1)=MAPbart(:,:,1,c1);
        %newsam=[newsam c1];
        weight(a1)=1/numSamples;
    end
    weight=weight/sum(weight);
end
