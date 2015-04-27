function pose=ReverseOdometry(u,pose0,alphas,theModel)

    switch lower(theModel)
        case 'odometrymotion'
            pose=ReverseOdometryMotion(u,pose0,alphas);
        case 'velocitymotion'
            pose=ReverseVelocityMotion(u,pose0,alphas);
    end
    
end