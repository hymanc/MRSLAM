function pose=SampleMotionModel(tv,rv,dt,pose0,M)

%Table 5.3

    tv=tv+M(1)*randn(1);
    rv=rv+M(2)*randn(1);
    gamma=M(3)*randn(1);
    
    if (rv~=0)
        vtwt=tv/rv;
    else
        vtwt=0;
    end

    pose=pose0+[vtwt*(-sin(pose0(3))+sin(pose0(3)+rv*dt));...
        vtwt*(cos(pose0(3))-cos(pose0(3)+rv*dt))+M(2)*randn(1);...
        rv*dt+gamma*dt];

    
    if (pose(3)>pi)
        pose(3)=pose(3)-2*pi;
    elseif pose(3)<-pi
        pose(3)=pose(3)+2*pi;
    end
end