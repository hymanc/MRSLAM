function [data,mT]=partitionDataAlbert(nRobots,posein,odomin,radin)

    sP=size(posein,2);
    vect=1:1:sP;
    
    data=struct('time',[],'pose',[],'odom',[],'rad',[]);
    
    type='EvenSplit';
    
    mT=-inf;
    for a1=1:nRobots
        
        switch lower(type)
            case 'follower'
                inds=vect(a1:nRobots:end);
            case 'evensplit'
                inds=vect(floor((1:floor(sP/nRobots))+floor(sP/nRobots)*(a1-1)));
        end
        
        
        data(a1).time=1:numel(inds);
        data(a1).pose=posein(:,inds);
        data(a1).odom=odomin(:,inds);
        data(a1).rad=radin(:,inds);
        mT=max([data(a1).time(end) mT]);
    end
end