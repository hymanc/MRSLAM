function pose=ReverseOdometryMotion(u,pose,alphas)

    r2=u(1)^2;
    th2=u(2)^2;
    dtheta=u(2)-(alphas(1)*th2+alphas(2)*r2)*randn(1);
    dx=u(1)-(alphas(4)*th2+alphas(3)*r2)*randn(1);
    
    pose(1)=pose(1)-dx*cos(pose(3));
    pose(2)=pose(2)-dx*sin(pose(3));
    pose(3)=pose(3)-dtheta;
    
    pose(3)=normalizeTheta(pose(3));

end