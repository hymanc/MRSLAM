function orderEncounters=parseEncounters(encounters,start)


    orderEncounters=[];
    for a1=1:size(encounters,1)
        for a2=1:numel(start)
            if (encounters(a1,2)==start(a2) & ~sum(start==encounters(a1,3)))
                
                start=[start;encounters(a1,3)];
                orderEncounters=[orderEncounters;encounters(a1,:)];
            end
        end
    end
end