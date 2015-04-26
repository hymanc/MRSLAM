function orderEncounters=parseEncounters(encounters,start,robotInds)
%Used to parse the encounters from start. 
%Example: We start from robot 1, robot 2 observes robot 4, however, robot 2
%and 4 do not communicate because they have not been joined yet.
%I am aware this is not how the algorithm actually works, but I am
%simplifying for now.

    orderEncounters=[];
    for a1=1:size(encounters,1)
        for a2=1:numel(start)
            if (encounters(a1,2)==start(a2) & ~sum(start==encounters(a1,3)) & sum((encounters(a1,3)==robotInds | encounters(a1,2)==robotInds))==2)
                start=[start;encounters(a1,3)];
                orderEncounters=[orderEncounters;encounters(a1,:)];
            end
        end
    end
end