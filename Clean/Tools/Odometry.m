function pose=Odometry(u,pose0,alphas,theModel)

    switch lower(theModel)
        case 'odometrymotion'
            pose=OdometryMotion(u,pose0,alphas);
        case 'velocitymotion'
            pose=velocityMotion(u,pose0,alphas);
    end
    
end