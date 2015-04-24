function pose=OdometryMotionReverse(u,pose,alpha)

    r2=u(1)^2;
    th2=u(2)^2;
    dtheta=u(2)-(alpha(1)*th2+alpha(2)*r2)*randn(1);
    dx=u(1)-(alpha(4)*th2+alpha(3)*r2)*randn(1);
 
    
    pose(1)=pose(1)-dx*cos(pose(3));
    pose(2)=pose(2)-dx*sin(pose(3));
    pose(3)=pose(3)-dtheta;

    if (abs(pose(3))>pi)
        if (pose(3)>pi)
            pose(3)=pose(3)-2*pi;
        else
            pose(3)=pose(3)+2*pi;
        end
    end
end