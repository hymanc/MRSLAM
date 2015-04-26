function pose=ReverseVelocityMotion(u,pose0,alphas)

%Table 5.3

    v2=u(1)^2;
    o2=u(2)^2;
    v=u(1)+(alphas(1)*v2+alphas(2)*o2)*randn(1);
    omega=u(2)+(alphas(3)*v2+alphas(4)*o2)*randn(1);
    gamma=(alphas(5)*v2+alphas(6)*o2)*randn(1);
    dt=u(3);
    
    if (omega~=0)
        vo=v/omega;
    else
        vo=0;
    end

    pose=pose0-[vo*(-sin(pose0(3))+sin(pose0(3)+omega*dt));...
        vo*(cos(pose0(3))-cos(pose0(3)+omega*dt))+M(2)*randn(1);...
        omega*dt+gamma*dt];

    pose(3)=normalizeTheta(pose(3));
end