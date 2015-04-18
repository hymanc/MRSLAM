function pose=Odometry(tv,rv,dt,pose0)

    if (rv~=0)
        vtwt=tv/rv;
    else
        vtwt=0;
    end

    pose=pose0+[vtwt*(-sin(pose0(3))+sin(pose0(3)+rv*dt));...
        vtwt*(cos(pose0(3))-cos(pose0(3)+rv*dt));...
        rv*dt];

    
    if (pose(3)>pi)
        pose(3)=pose(3)-2*pi;
    elseif pose(3)<-pi
        pose(3)=pose(3)+2*pi;
    end
end