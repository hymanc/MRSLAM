function [pose,weight] = resampleSingle(pose,weight)

    numSamples=numel(weight);
    Xbart=pose;
    W=cumsum(weight);
    s0=1/numSamples*rand;
    s=mod(linspace(s0,1+s0,numSamples),1);
    %newsam=[];
    for a1=1:numSamples
        c1=1;
        while (s(a1)>W(c1))
            c1=c1+1;
        end
        pose(a1,:)=Xbart(c1,:);
        %newsam=[newsam c1];
        weight(a1)=1/numSamples;
    end
    weight=weight/sum(weight);
end