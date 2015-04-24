
useParallel=true;
if (useParallel)
    myCluster = parcluster('local');
    myCluster.NumWorkers = 7;
    saveAsProfile(myCluster,'local');
    ncores=matlabpool('size');
    if (ncores~=myCluster.NumWorkers & ncores>1)
        matlabpool('close');
        matlabpool(myCluster.NumWorkers);
    elseif (ncores~=myCluster.NumWorkers)
        matlabpool(myCluster.NumWorkers);
    end
end

X0=[150 25 25 275 275 60 60  230 230 150;...
    150 25 275 25 275 90 220 90 220 225];



pfilled=0.95;
NROBOTS=10;
NTRIALS=100;
if (~exist('TIME'))
    TIME=zeros(NROBOTS,NTRIALS);
end
%for a1=NROBOTS:-1:1
for a1=1:NROBOTS
    parfor a2=1:NTRIALS
        if (TIME(a1,a2)==0)
            fprintf('Robots: %d, Trial: %d\n',a1,a2);
            TIME(a1,a2)=scanAndFill95percent(a1,pfilled,X0(:,1:a1));
            fprintf('\n')
        end
    end
    save('TIME.mat','TIME')        
end


figure(1231)
for a1=1:NROBOTS
    muT(a1)=mean(TIME(a1,TIME(a1,:)<1000),2);
    stdT(a1)=std(TIME(a1,TIME(a1,:)<1000),[],2);
end
    plot(muT,'k');
    hold on;
    for a2=1:numel(muT)
        plot([a2 a2],[muT(a2)+stdT(a2) muT(a2)-stdT(a2)],'b-')
        plot([a2 a2],[muT(a2)+stdT(a2) muT(a2)-stdT(a2)],'bo')
    end
    hold off;

save('TIME.mat','TIME')
