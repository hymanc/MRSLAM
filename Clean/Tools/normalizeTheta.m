function theta=normalizeTheta(theta)

    if (abs(theta)>pi)
        if (theta>pi)
            theta=theta-2*pi;
        else
            theta=theta+2*pi;
        end
    end

end